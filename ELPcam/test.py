import cv2

# 打开USB摄像头（索引0，若有多个摄像头可改成1、2…）
cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()  # 取一帧
    if not ret:
        break

    cv2.imshow("USB Camera", frame)  # 显示画面

    # 按 q 或 ESC 退出
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:
        break

cap.release()
cv2.destroyAllWindows()
