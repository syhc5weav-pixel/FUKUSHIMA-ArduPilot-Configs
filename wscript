
# encoding: utf-8

def build(bld):
    bld.ap_library(
        'AP_SX1280_AJ',
        use=['AP_HAL', 'AP_Math', 'GCS_MAVLink'],
    )
