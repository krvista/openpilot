"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params


# Extra lateral look-ahead for the driving model only (its action time / lateral_control_params), on top of the
# lateral delay. Controls keep the plain delay. WK2 drivelog 8f-94 (1af62ef): on straights the car weaves at ~0.18 Hz
# (+/-0.2 deg, +/-3-4 cm) on every build since 23cf66e; at that frequency the loop from the controls' request to the
# steering-angle lateral accel delivers 0.65-0.9 of it with 0.37-0.73 s effective lag, vs 0.27-0.31 s learned by
# lagd, worse at 49-65 km/h. A linear loop model puts the weave 10-17% lower with 0.1-0.2 s more model look-ahead.
# Experiment: compare the 0.1-0.3 Hz lane-offset / angle power on straights and curve entry timing before vs after.
MODEL_LAT_LEAD = {
  'JEEP_GRAND_CHEROKEE_2019': 0.12,
}


def get_model_lat_lead(car_fingerprint: str) -> float:
  return MODEL_LAT_LEAD.get(str(car_fingerprint), 0.)


def get_lat_delay(params: Params, stock_lat_delay: float) -> float:
  if params.get_bool("LagdToggle"):
    return float(params.get("LagdValueCache", return_default=True))

  return stock_lat_delay
