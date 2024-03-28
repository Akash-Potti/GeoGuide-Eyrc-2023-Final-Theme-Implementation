##################### EVENT CLASSIFICATION #####################
'''
 * Team Id: 1792
 * Author List: Akash Potti
 * Filename: detection.py
 * Theme: GeoGuide
 * Functions: detection()
 * Global Variables: identified_labels
 '''
####################### IMPORT MODULES #######################
import cv2
import torch.nn as nn
import torchvision.transforms as T
import torch
import torch.nn.functional as F
import numpy as np
from torchvision import models
####################### FUNCTION DEFINITION #######################
'''
* Function Name: detection
* Input: None
* Output: identified_labels (dict) - A dictionary containing the identified labels mapped to their corresponding regions of interest (ROIs).
* Logic: This function performs real-time object detection using a pretrained neural network model. It captures video frames from a camera feed, preprocesses each frame, identifies contours, filters them based on size and aspect ratio, extracts ROIs, and passes them through the neural network for classification. If the confidence of the prediction is above a certain threshold, the label is recorded along with its corresponding ROI. The identified labels are continuously updated and written to a text file.
* Example Call: 
  identified_labels = detection()

'''


def detection():
    identified_labels = {}

    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.get(cv2.CAP_PROP_ZOOM)

    x_frame = 400
    y_frame = 50
    width = 950
    height = 950

    filtered_contours = []
    filtered_contours_x = []
    filtered_contours_y = []
    filtered_contours_w = []
    filtered_contours_h = []

    transform = T.Compose([T.ToPILImage(), T.Resize((85, 85)), T.ToTensor(
    ), T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])

    class_names = ["combat", "humanitarian_aid",
                   "military_vehicles", "fire", "destroyed_buildings", ""]
    class_label_mapping = {
        "fire": "Fire",
        "destroyed_buildings": "Destroyed buildings",
        "combat": "Combat",
        "humanitarian_aid": "Humanitarian Aid and rehabilitation",
        "military_vehicles": "Military Vehicles",
        "": ""
    }

    model = models.efficientnet_b2(pretrained=True)
    num_classes = 6
    model.classifier = nn.Linear(1408, num_classes)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)
    model.load_state_dict(torch.load('model.pth'))
    model.eval()

    while True:
        label_maps = {0: "A", 1: "B", 2: "C", 3: "D", 4: "E"}

        ret, frame = cap.read()
        frame = cv2.resize(frame, None, fx=2.25, fy=2.25)
        frame = frame[y_frame:y_frame+height, x_frame:x_frame+width]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        sharpen_kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
        sharpen = cv2.filter2D(gray, -1, sharpen_kernel)
        _, threshold = cv2.threshold(sharpen, 200, 255, cv2.THRESH_BINARY)
        countours, _ = cv2.findContours(
            threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if (len(filtered_contours) == 0):
            for cnt in countours:
                area = cv2.contourArea(cnt)
                if area > 4800 and area < 8000:
                    x, y, w, h = cv2.boundingRect(cnt)
                    aspect_ratio = float(w) / h

                    # Check if the aspect ratio is close to 1 (indicating a square)
                    if 0.9 <= aspect_ratio <= 1.1:
                        filtered_contours.append(cnt)
                        filtered_contours_x.append(x)
                        filtered_contours_y.append(y)
                        filtered_contours_w.append(w)
                        filtered_contours_h.append(h)

        for cnt in range(len(filtered_contours)):
            x = filtered_contours_x[cnt]
            y = filtered_contours_y[cnt]
            w = filtered_contours_w[cnt]
            h = filtered_contours_h[cnt]
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3)

        for i, contour in enumerate(filtered_contours):
            x, y, w, h = cv2.boundingRect(contour)
            x += 3
            y += 3
            w -= 6
            h -= 6
            roi = frame[y:y+h, x:x+w]
            roi_tensor = transform(roi).unsqueeze(0).to(device)

            with torch.no_grad():
                output = model(roi_tensor)
                probabilities = F.softmax(output, dim=1)
                confidence, predicted_class = torch.max(probabilities, 1)
                confidence = confidence.item()
                if confidence > 0.5:
                    prediction_label = class_names[predicted_class.item()]
                    if prediction_label.lower() != "":
                        identified_labels[label_maps[i]] = class_label_mapping.get(
                            prediction_label, "")

                cv2.putText(frame, prediction_label, (x, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                mapped_label = class_label_mapping.get(prediction_label, "")
                print(identified_labels)
            cv2.imwrite(f'{i}.jpeg', roi)

        cv2.imshow('Arena Feed', frame)
        # cv2.imshow('Arena Feed', threshold)

        if cv2.waitKey(1) == ord('q'):
            break
        with open('identified_labels.txt', 'w') as file:
            for key, value in identified_labels.items():
                file.write(f'{key}: {value}\n')

    return identified_labels


if __name__ == "__main__":
    identified_labels = detection()
    print(identified_labels)
