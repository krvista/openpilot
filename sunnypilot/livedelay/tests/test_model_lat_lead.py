from opendbc.car.chrysler.values import CAR as CHRYSLER
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.sunnypilot.livedelay.helpers import get_model_lat_lead


def test_model_lat_lead_only_for_listed_cars():
  assert get_model_lat_lead(CHRYSLER.JEEP_GRAND_CHEROKEE_2019) == 0.12
  assert get_model_lat_lead(CHRYSLER.JEEP_GRAND_CHEROKEE) == 0.
  assert get_model_lat_lead(TOYOTA.TOYOTA_RAV4) == 0.
  assert get_model_lat_lead("") == 0.
