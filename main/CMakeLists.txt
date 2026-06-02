# 這是我們專案 "main" 的唯一設定檔。
#
# SRCS: 列出我們所有的 .cpp 原始檔
# REQUIRES: 列出我們需要的 "內建" 組件 (esp_matter 和 app_reset)
#
idf_component_register(SRCS "app_main.cpp" "app_driver.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_matter app_reset led_strip driver espressif__button)