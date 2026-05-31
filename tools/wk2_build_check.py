#!/usr/bin/env python3
"""Extract the build identity (git commit / version / branch) that recorded each rlog,
from the initData message. Confirms whether the logs were produced by the
sunny-tizi-v6 code we are editing. Writes JSON.

Usage: PYTHONPATH=$PWD python3 tools/wk2_build_check.py <out.json> <rlog glob...>
"""
import sys, glob, os, json
import zstandard, capnp

capnp.remove_import_hook()
_CD = os.path.join(os.getcwd(), 'cereal')
_LOG = capnp.load(os.path.join(_CD, 'log.capnp'), imports=[_CD, os.getcwd()])


def read_events(path):
    raw = open(path, 'rb').read()
    data = zstandard.ZstdDecompressor().decompress(raw, max_output_size=400 * 1024 * 1024)
    for ev in _LOG.Event.read_multiple_bytes(data):
        yield ev


def first_initdata(path):
    for ev in read_events(path):
        if ev.which() == 'initData':
            d = ev.initData
            out = {}
            for k in ('version', 'gitCommit', 'gitBranch', 'gitRemote', 'dirty', 'gitCommitDate'):
                try:
                    out[k] = str(getattr(d, k))
                except Exception:
                    pass
            # params often carry GitCommit / SmartCruiseControl* toggles
            try:
                params = {}
                for e in d.params.entries:
                    key = e.key
                    if key in ('GitCommit', 'GitBranch', 'GitCommitDate', 'Version',
                               'SmartCruiseControlVision', 'SmartCruiseControlMap',
                               'SmartCruiseControlVisionLowSpeed'):
                        try:
                            params[key] = bytes(e.value).decode('utf-8', 'replace')
                        except Exception:
                            params[key] = repr(e.value)
                out['params'] = params
            except Exception as ex:
                out['params_err'] = repr(ex)[:80]
            return out
    return None


def main(argv):
    out_path, srcs = argv[0], argv[1:]
    ins = []
    for a in srcs:
        ins += sorted(glob.glob(a)) or [a]
    res = {}
    for src in ins:
        try:
            res[os.path.basename(src)] = first_initdata(src)
        except Exception as e:
            res[os.path.basename(src)] = {'error': repr(e)[:120]}
    with open(out_path, 'w') as f:
        json.dump(res, f, indent=2)
    print(f"wrote {out_path}: {len(res)} segs")


if __name__ == '__main__':
    main(sys.argv[1:])
