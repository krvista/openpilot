#!/usr/bin/env bash
# drivevideo.sh - comma 디바이스에서 주행 영상의 필요한 구간만 꺼내 온다.
#
# 분석에서 "이 경로의 몇 분 몇 초 영상이 필요하다"고 나온 구간을 받아, 구간별 mp4 한 개로 잘라
# 출력 폴더에 모은다. drivelog.sh 가 옆에 있으면 --upload 로 데이터 브랜치 루트에 바로 올린다.
#
# 사용법 (Git Bash / WSL, drivelog.sh 와 같은 PC):
#   ./drivevideo.sh list                         # 요청 목록과 우선순위를 보여준다
#   ./drivevideo.sh get --top 2                  # 우선순위 위에서 2 개 구간
#   ./drivevideo.sh get --all                    # 목록 전부
#   ./drivevideo.sh get 000000b4 36:10-38:50     # 임의 구간 (여러 쌍을 이어 써도 된다)
# 옵션:
#   --host comma@IP    디바이스 주소 (기본: drivelog.conf 의 PROFILE_SSH[ccnc], 없으면 아래 기본값)
#   --cam f|q|e        f = 전방 카메라 원본(fcamera.hevc, 기본), q = 저해상도(qcamera.ts), e = 광각(ecamera.hevc)
#   --width N          내보낼 가로 픽셀 (기본 1280; 0 = 원본 크기)
#   --crf N            화질 (기본 26; 작을수록 좋고 크다)
#   --raw              자르지 않고 세그먼트 파일을 그대로 가져온다 (ffmpeg 이 없을 때 자동)
#   --out DIR          출력 폴더 (기본 ./drivevideo_out)
#   --from-dir DIR     디바이스 대신 이미 받아 둔 realdata 폴더에서 읽는다
#   --upload           끝나면 ./drivelog.sh put 으로 올린다 (파일당 95 MB 를 넘으면 건너뛴다)
#   --dry-run          무엇을 가져올지만 보여준다
#
# 시각은 경로 시작부터의 mm:ss (보고서와 같은 기준). 세그먼트는 60 초 단위다.
# 파일 이름: video_<경로>_<시작mmss>-<끝mmss>_<cam>.mp4  (--raw 면 video_<경로>_seg<N>_<cam>.<확장자>)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEVICE_DATA_DIR="/data/media/0/realdata"
HOST="comma@192.168.1.135"
CAM="f"; WIDTH=1280; CRF=26; RAW=0; OUT="./drivevideo_out"; FROM_DIR=""; UPLOAD=0; DRY=0; TOP=0; ALL=0
FPS=20

# 우선순위 순. "경로 시작-끝 설명"
WINDOWS=(
  "000000b4 36:10-38:50 106 km/h 우커브에서 24 cm 바깥: 차선·차로 폭·도로 기울기"
  "000000b6 46:25-47:05 램프(43 km/h)에서 안쪽 54-71 cm: 안쪽 실선·연석까지 여유"
  "000000b5 18:25-19:05 96 km/h 좌커브 진입에서 2 초에 23 cm 쏠림: 이음매·트럭·바람"
  "000000b4 18:43-19:05 41 km/h 커브 안쪽 18-27 cm: 안쪽 선·구조물까지 거리"
  "000000b6 68:15-70:40 좌우 커브 연속: 양쪽 모두 왼쪽인지, 우커브에서만 바깥인지"
  "000000b6 13:35-15:22 31-44 km/h 굽은 국도: 중앙선·길 가장자리 여유, 차선이 실제로 넓어지는지"
  "000000b6 49:10-50:47 평범한 직선의 13 cm 쏠림에 보이는 원인이 있는지, 우커브 진입 계단"
  "000000b5 31:40-31:56 41 km/h 급커브 안쪽 56-68 cm: 안쪽 선을 넘는지"
  "000000b4 41:00-41:25 107 km/h 우커브 16 cm 바깥: 선행차 주행선·옆 차"
  "000000b6 93:10-94:00 102-104 km/h 에서 17 cm 쏠림 두 번: 옆 트럭·커브 탈출부"
  "000000b5 14:31-16:34 좌커브·우커브·직선 연속: 차선과 보닛 간격의 좌우 차이"
  "000000b4 08:55-09:12 31 km/h 좌커브 안쪽 40 cm"
  "000000b1 11:55-12:16 문제없는 42 km/h 좌커브 (비교 기준)"
  "000000b0 10:25-10:56 17-33 km/h 완만한 좌커브 안쪽 20 cm"
)

log() { printf '%s\n' "$*" >&2; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }
to_s() { local m="${1%%:*}" s="${1##*:}"; echo $((10#$m * 60 + 10#$s)); }
tag() { printf '%s' "$1" | tr -d ':'; }

if [ -f "$SCRIPT_DIR/drivelog.conf" ]; then
  declare -A PROFILE_SSH PROFILE_BRANCH PROFILE_DONGLE
  # shellcheck disable=SC1091
  . "$SCRIPT_DIR/drivelog.conf" || true
  [ -n "${PROFILE_SSH[ccnc]:-}" ] && HOST="${PROFILE_SSH[ccnc]}"
fi

CMD="${1:-list}"; shift || true
REQ=()
while [ $# -gt 0 ]; do
  case "$1" in
    --host) HOST="$2"; shift 2 ;;
    --cam) CAM="$2"; shift 2 ;;
    --width) WIDTH="$2"; shift 2 ;;
    --crf) CRF="$2"; shift 2 ;;
    --raw) RAW=1; shift ;;
    --out) OUT="$2"; shift 2 ;;
    --from-dir) FROM_DIR="$2"; shift 2 ;;
    --upload) UPLOAD=1; shift ;;
    --dry-run) DRY=1; shift ;;
    --top) TOP="$2"; shift 2 ;;
    --all) ALL=1; shift ;;
    -h|--help) sed -n '2,24p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) die "알 수 없는 옵션: $1" ;;
    *) [ $# -ge 2 ] || die "경로와 구간을 쌍으로 줄 것: 000000b4 36:10-38:50"
       REQ+=("$1 $2"); shift 2 ;;
  esac
done
case "$HOST" in *@*) ;; *) HOST="comma@$HOST" ;; esac
case "$CAM" in
  f) CAMFILE="fcamera.hevc" ;; e) CAMFILE="ecamera.hevc" ;; q) CAMFILE="qcamera.ts" ;;
  *) die "--cam 은 f, q, e 중 하나" ;;
esac

if [ "$CMD" = "list" ]; then
  i=1
  for w in "${WINDOWS[@]}"; do printf '%2d  %s\n' "$i" "$w"; i=$((i + 1)); done
  exit 0
fi
[ "$CMD" = "get" ] || die "명령은 list 또는 get"

if [ ${#REQ[@]} -eq 0 ]; then
  [ "$ALL" = 1 ] && TOP=${#WINDOWS[@]}
  [ "$TOP" -gt 0 ] || die "--top N, --all, 또는 '경로 mm:ss-mm:ss' 를 줄 것"
  i=0
  for w in "${WINDOWS[@]}"; do
    [ "$i" -lt "$TOP" ] || break
    REQ+=("$(printf '%s' "$w" | cut -d' ' -f1,2)"); i=$((i + 1))
  done
fi

HAVE_FFMPEG=1
command -v ffmpeg >/dev/null 2>&1 || { HAVE_FFMPEG=0; RAW=1; log "ffmpeg 이 없어 세그먼트 파일을 그대로 가져온다 (--raw)."; }

# 디바이스(또는 --from-dir)의 세그먼트 폴더 목록: <경로>--<해시>--<번호>
if [ -n "$FROM_DIR" ]; then
  [ -d "$FROM_DIR" ] || die "폴더 없음: $FROM_DIR"
  DIRS="$(cd "$FROM_DIR" && ls -1)"
else
  DIRS="$(ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10 "$HOST" "ls -1 $DEVICE_DATA_DIR")" \
    || die "디바이스에 접속할 수 없다: $HOST (설정에서 SSH 가 켜져 있는지, 같은 네트워크인지 확인)"
fi

seg_dir() {   # <경로> <세그먼트 번호> -> 폴더 이름 (없으면 빈 문자열)
  printf '%s\n' "$DIRS" | grep -E "^$1--[0-9a-f]+--$2\$" | head -1 || true
}

fetch() {     # <폴더 이름> <받을 경로>
  if [ -n "$FROM_DIR" ]; then
    cp -- "$FROM_DIR/$1/$CAMFILE" "$2"
  else
    scp -q -o BatchMode=yes -o ConnectTimeout=10 "$HOST:$DEVICE_DATA_DIR/$1/$CAMFILE" "$2"
  fi
}

mkdir -p "$OUT"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
MADE=(); MISSING=0

for r in "${REQ[@]}"; do
  route="${r%% *}"; win="${r##* }"
  a="${win%%-*}"; b="${win##*-}"
  t0=$(to_s "$a"); t1=$(to_s "$b")
  [ "$t1" -gt "$t0" ] || die "구간이 잘못됐다: $r"
  s0=$((t0 / 60)); s1=$(((t1 - 1) / 60))
  log ""
  log "== $route $a-$b  (세그먼트 $s0..$s1, $CAMFILE)"
  parts=(); ok=1
  for s in $(seq "$s0" "$s1"); do
    d="$(seg_dir "$route" "$s")"
    if [ -z "$d" ]; then log "   세그먼트 $s 가 디바이스에 없다 (이미 지워졌을 수 있다)"; ok=0; continue; fi
    if [ "$DRY" = 1 ]; then log "   가져올 것: $d/$CAMFILE"; continue; fi
    f="$TMP/${route}_${s}.${CAMFILE##*.}"
    if fetch "$d" "$f" 2>/dev/null && [ -s "$f" ]; then
      parts+=("$f"); log "   받음: $d/$CAMFILE ($(($(wc -c < "$f") / 1048576)) MB)"
    else
      log "   $d 에 $CAMFILE 이 없다"; ok=0
    fi
  done
  [ "$ok" = 1 ] || MISSING=$((MISSING + 1))
  [ "$DRY" = 1 ] && continue
  [ ${#parts[@]} -gt 0 ] || continue

  if [ "$RAW" = 1 ]; then
    for f in "${parts[@]}"; do
      s="$(basename "$f")"; s="${s#"${route}"_}"; s="${s%.*}"
      o="$OUT/video_${route}_seg${s}_${CAM}.${CAMFILE##*.}"
      mv -- "$f" "$o"; MADE+=("$o"); log "   -> $o"
    done
    continue
  fi

  # 세그먼트가 하나라도 빠졌으면 시각이 어긋나므로 잘라 붙이지 않고 있는 것만 통째로 낸다
  if [ "$ok" != 1 ]; then
    for f in "${parts[@]}"; do
      s="$(basename "$f")"; s="${s#"${route}"_}"; s="${s%.*}"
      o="$OUT/video_${route}_seg${s}_${CAM}.mp4"
      if [ "$CAM" = q ]; then in=(-i "$f"); else in=(-r "$FPS" -i "$f"); fi
      ffmpeg -hide_banner -loglevel error -y "${in[@]}" -an -c:v libx264 -preset veryfast -crf "$CRF" \
        $([ "$WIDTH" -gt 0 ] && [ "$CAM" != q ] && echo "-vf scale=$WIDTH:-2") "$o"
      MADE+=("$o"); log "   -> $o (세그먼트 전체)"
    done
    continue
  fi

  cat "${parts[@]}" > "$TMP/join.${CAMFILE##*.}"
  off=$((t0 - s0 * 60)); dur=$((t1 - t0))
  o="$OUT/video_${route}_$(tag "$a")-$(tag "$b")_${CAM}.mp4"
  if [ "$CAM" = q ]; then in=(-i "$TMP/join.ts"); else in=(-r "$FPS" -i "$TMP/join.hevc"); fi
  vf=()
  if [ "$WIDTH" -gt 0 ] && [ "$CAM" != q ]; then vf=(-vf "scale=$WIDTH:-2"); fi
  ffmpeg -hide_banner -loglevel error -y "${in[@]}" -ss "$off" -t "$dur" -an "${vf[@]}" \
    -c:v libx264 -preset veryfast -crf "$CRF" -pix_fmt yuv420p "$o"
  rm -f "$TMP"/join.* "${parts[@]}"
  MADE+=("$o"); log "   -> $o ($(($(wc -c < "$o") / 1048576)) MB)"
done

[ "$DRY" = 1 ] && exit 0
log ""
log "만든 파일 ${#MADE[@]} 개, 폴더 $OUT"
[ "$MISSING" -gt 0 ] && log "세그먼트가 빠진 구간 $MISSING 개 (위 메시지 참고)"
[ ${#MADE[@]} -gt 0 ] || exit 1

if [ "$UPLOAD" = 1 ]; then
  DL="$SCRIPT_DIR/drivelog.sh"
  [ -x "$DL" ] || die "--upload 는 drivelog.sh 가 이 스크립트와 같은 폴더에 있어야 한다. 직접 올리려면: ./drivelog.sh put $OUT/video_*"
  UP=()
  for f in "${MADE[@]}"; do
    if [ "$(wc -c < "$f")" -gt $((95 * 1048576)) ]; then
      log "건너뜀(95 MB 초과): $f  -> --width 를 줄이거나 --crf 를 키워 다시 만들 것"
    else
      UP+=("$f")
    fi
  done
  [ ${#UP[@]} -gt 0 ] && "$DL" put "${UP[@]}"
else
  log "올리려면: ./drivelog.sh put ${OUT}/video_*   (drivelog.sh 가 있는 폴더에서)"
fi
