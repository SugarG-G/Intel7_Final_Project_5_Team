DT_NAME := vl53_dual
BUILD_DIR := build

overlay:
	mkdir -p $(BUILD_DIR)
	dtc -@ -I dts -O dtb -o $(BUILD_DIR)/$(DT_NAME).dtbo vl53_dual_overlay.dts

clean:
	rm -rf $(BUILD_DIR)

.PHONY: overlay clean
