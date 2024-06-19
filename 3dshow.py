import cv2
import matplotlib.pyplot as plt
import mediapipe as mp
import time
import numpy as np

mp_pose = mp.solutions.holistic
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

colorclass = plt.cm.ScalarMappable(cmap='jet')
colors = colorclass.to_rgba(np.linspace(0, 1, int(33)))
colormap = (colors[:, 0:3])


def draw3d(plt, ax, world_landmarks, connnection=mp_pose.HAND_CONNECTIONS):
    ax.clear()
    ax.set_xlim3d(0.2, 0.8)
    ax.set_ylim3d(-0.5, 0.1)
    ax.set_zlim3d(-0.5, 0.1)

    landmarks = []
    for index, landmark in enumerate(world_landmarks.landmark):
        landmarks.append([landmark.x, landmark.z, landmark.y * (-1)])
    landmarks = np.array(landmarks)

    ax.scatter(landmarks[:, 0], landmarks[:, 1], landmarks[:, 2], s=50)
    for _c in connnection:
        ax.plot([landmarks[_c[0], 0], landmarks[_c[1], 0]],
                [landmarks[_c[0], 1], landmarks[_c[1], 1]],
                [landmarks[_c[0], 2], landmarks[_c[1], 2]], 'k')

    plt.pause(0.001)


# 端口号一般是0，除非你还有其他摄像头
# 使用本地视频推理，复制其文件路径代替端口号即可
cap = cv2.VideoCapture(0)
with mp_pose.Holistic(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        model_complexity=1) as pose:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
            continue

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        start = time.time()
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = pose.process(image)

        # Draw the pose annotation on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        mp_drawing.draw_landmarks(
            image,
            results.right_hand_landmarks,
            mp_pose.HAND_CONNECTIONS,
            landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

        end = time.time()
        fps = 1 / (end - start)
        fps = "%.2f fps" % fps
        # 实时显示帧数
        image = cv2.flip(image, 1)
        cv2.putText(image, "FPS {0}".format(fps), (100, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 3)

        cv2.imshow('MediaPipe Pose', image)
        if cv2.waitKey(5) & 0xFF == 27:
            break
        if results.right_hand_landmarks:
            draw3d(plt, ax, results.right_hand_landmarks)

cap.release()
