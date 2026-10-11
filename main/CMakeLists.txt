idf_component_register(
    SRCS "main.cpp" "wake_word.cpp" "ssd1306.cpp" "eyes.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_timer
)
