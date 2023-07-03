import cv2
import pyrealsense2 as rs
import numpy as np
import torch


model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

m = 0
b = 0


while True:

    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame:
        continue

    color_image = np.asanyarray(color_frame.get_data())

    image = cv2.flip(color_image, 1)

    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    results_yolo = model(image)

    h, w, _ = image.shape

    


    for detection in results_yolo.xyxy[0]:

        x1_rect, y1_rect, x2_rect, y2_rect, confidence, class_id = detection
        label = model.names[int(class_id)]
        x_milieu = (x1_rect+x2_rect)/2
        y_milieu = (y1_rect + y2_rect)/2
        z_milieu = depth_frame.get_distance(int(x_milieu), int(y_milieu))

        coord_milieu = (int(x_milieu),int(y_milieu))


        if  label in ["suitcase", "handbag", "backpack"]:

            cv2.rectangle(image, (int(x1_rect), int(y1_rect)), (int(x2_rect), int(y2_rect)), (0, 255, 0), 2)                            
            text = cv2.putText(image, "{0} at {1:.2f}m".format(label, z_milieu) , (int(x1_rect), int(y2_rect + 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)        

            print("{0} at {1:.2f}m".format(label, z_milieu))
        

    cv2.imshow('Grasp bag', image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()