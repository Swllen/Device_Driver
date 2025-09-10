import cv2

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Windows 建议 CAP_DSHOW

prop_map = {
    "FRAME_WIDTH": cv2.CAP_PROP_FRAME_WIDTH,
    "FRAME_HEIGHT": cv2.CAP_PROP_FRAME_HEIGHT,
    "FPS": cv2.CAP_PROP_FPS,
    "FOURCC": cv2.CAP_PROP_FOURCC,
    "BRIGHTNESS": cv2.CAP_PROP_BRIGHTNESS,
    "CONTRAST": cv2.CAP_PROP_CONTRAST,
    "SATURATION": cv2.CAP_PROP_SATURATION,
    "HUE": cv2.CAP_PROP_HUE,
    "GAIN": cv2.CAP_PROP_GAIN,
    "EXPOSURE": cv2.CAP_PROP_EXPOSURE,
    "GAMMA": cv2.CAP_PROP_GAMMA,
    "TEMPERATURE": cv2.CAP_PROP_TEMPERATURE,
    "SHARPNESS": cv2.CAP_PROP_SHARPNESS,
    "BACKLIGHT": cv2.CAP_PROP_BACKLIGHT,
    "AUTOFOCUS": cv2.CAP_PROP_AUTOFOCUS,
    "FOCUS": cv2.CAP_PROP_FOCUS,
    "AUTO_EXPOSURE": cv2.CAP_PROP_AUTO_EXPOSURE,
    "AUTO_WB": cv2.CAP_PROP_AUTO_WB,
    "WB_TEMPERATURE": cv2.CAP_PROP_WB_TEMPERATURE,
    "ZOOM": cv2.CAP_PROP_ZOOM,
    "PAN": cv2.CAP_PROP_PAN,
    "TILT": cv2.CAP_PROP_TILT,
    "ROLL": cv2.CAP_PROP_ROLL,
    "IRIS": cv2.CAP_PROP_IRIS,
    "BUFFERSIZE": cv2.CAP_PROP_BUFFERSIZE,
    "CONVERT_RGB": cv2.CAP_PROP_CONVERT_RGB,
}

# 过滤掉不支持的属性
supported = {}
for i, (name, prop) in enumerate(prop_map.items()):
    val = cap.get(prop)
    if val == -1:
        continue
    supported[i] = (name, prop)
    print(f"[{i:2d}] {name:15s} : {val}")

print("\n输入编号和新值 (例如: 3 640)，输入 q 退出。\n")

while True:
    user_in = input(">>> ")
    if user_in.lower() in ("q", "quit", "exit"):
        break

    try:
        idx, val = user_in.split()
        idx = int(idx)
        val = float(val)  # OpenCV set() 接收 float
    except Exception as e:
        print("输入格式错误，应为: 编号 数值")
        continue

    if idx not in supported:
        print("无效编号")
        continue

    name, prop = supported[idx]
    ok = cap.set(prop, val)
    new_val = cap.get(prop)
    print(f"设置 {name} = {val}, 结果: {'成功' if ok else '失败'}, 实际值 = {new_val}")

cap.release()
