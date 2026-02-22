# encoding: utf-8

def build(bld):
    bld.ap_library(
        name='AP_SX1280_AJ',
        dynamic_source='modules/waf/**',
        use=['AP_HAL', 'AP_Math', 'GCS_MAVLink'],
    )
