#libraries

from importlib.resources import path
import time
import torch
from matplotlib import pyplot as plt
import cv2 as cv
from math import atan2, cos, sin, sqrt, pi , tan , radians
import numpy as np
from fastsam import FastSAM, FastSAMPrompt
import torch 
import xml.etree.ElementTree as ET
import os
from shapely.geometry import LineString,Polygon
import pandas as pd
import client
import serial

import os
import cv2
import keyboard



def save_image(image, folder_path, filename):

    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    

    file_path = os.path.join(folder_path, filename)
    cv2.imwrite(file_path, image)
    print(f"saved: {file_path}")


#draw bounding box using yolo
def image_reading(IMAGE_PATH):
    image=cv.imread(IMAGE_PATH)
    return image

def YOLO_bbox(IMAGE_PATH):
    #model=torch.hub.load('ultralytics/yolov5','costum',path='bestfinal.pt',force_reload=True)
    model = torch.hub.load('ultralytics/yolov5:v7.0', 'custom',path='bestfinn.pt')
    #image=cv.imread(IMAGE_PATH)
    #image=cv.resize(image,(640,640))
    result=model(IMAGE_PATH)
    result.print()
    result.show()
    no_of_objects=np.size(result.pandas().xyxy,1)
    return no_of_objects,result

def create_data_array(no_of_objects,result):
    data = []
    for x in range(0,no_of_objects):
        row = []
        for x in range(0,6):
            row.append(0)
        data.append(row)
    return data


def object_label(data,result,no_of_objects):
    df=pd.DataFrame(result.pandas().xyxy[0])
    print(result.pandas().xyxy)
    for k in range(no_of_objects):
        data[k][0]=(df['name'][k])
    return data

def FastSaam_segmentation(result,no_of_objects,IMAGE_PATH):
    model = FastSAM('FastSAM-x.pt')
    DEVICE = torch.device(
        "cuda"
        if torch.cuda.is_available()
        else "mps"
        if torch.backends.mps.is_available()
        else "cpu"
    )
    everything_results = model(
        IMAGE_PATH,
        device=DEVICE,
        retina_masks=True,
        imgsz=1024,
        conf=0.4,
        iou=0.9,
    )
    prompt_process = FastSAMPrompt(IMAGE_PATH, everything_results, device=DEVICE)
    processed_bbox=[]
    ann=[]
    for i in range (no_of_objects):
        print(i)
        boudningbox=[0]
        boudningbox=[int(result.xyxy[0][i][0]),int(result.xyxy[0][i][1]),int(result.xyxy[0][i][2]),int(result.xyxy[0][i][3])]
        #processed_bbox=(np.array(boudningbox))
        ann2 = prompt_process.box_prompt(bbox=boudningbox)
        print(np.shape(ann2))
        ann.append(ann2[0])
    print("ann")
    ann=np.array(ann)
    prompt_process.plot(
        annotations=ann,
        output_path='./output/test5.JPG',
        mask_random_color=True,
        better_quality=True,
        retina=False,
        withContours=True,
    )
    return ann




#rotation angel calculation
def calculate_angle_and_coordinates(ann,result,data,image,no_of_objects):
    size=np.size(ann,0)
    for i in range(size):
        bw=ann[i]
        bw=np.array(bw, np.uint8)
        contours, _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
        for k, c in enumerate(contours):
            area = cv.contourArea(c)
            rect = cv.minAreaRect(c)
            box = cv.boxPoints(rect)
            box = np.int0(box)
            center = (int(rect[0][0]),int(rect[0][1])) 
            width = int(rect[1][0])
            height = int(rect[1][1])
            angle = int(rect[2])
            actual_angle=angle
            if width > height:
                angle = 90-angle
            else:
                angle=-angle
            data[i][4]=angle
            data[i][1:3]=center
            x=int(rect[0][0])
            y=int(rect[0][1])
            x_limit=list(range(x-400,x+400))
            x_limit=np.array(x_limit)
            y_limit=[]
            if (angle != 90 or angle!=-90):
                for k in range(-400,400):
                    y_limit.append(y-tan( radians(angle))*k)
            else:
                for k in range(-400,400):
                    y_limit.append(y)
            line = []
            for x in range(0,np.size(x_limit)):
                row = []
                for x in range(2):
                    row.append(0)
                line.append(row)
            c=np.reshape(c,(-1,2))
            line_1=LineString(np.column_stack((x_limit,y_limit)))
            line_2=Polygon(c)
            intersection=line_1.intersection(line_2)
            x_1=intersection.xy[0][0]
            x_2=intersection.xy[0][-1]
            y_1=intersection.xy[1][0]
            y_2=intersection.xy[1][-1]
            center=((int((int(x_1)+int(x_2))/2)),int((int(y_1)+int(y_2))/2))
            gripper_width=(sqrt((x_1-x_2)**2+(y_1-y_2)**2))
            offset = np.load("offset.npy")
            data[i][-1]=gripper_width
            [x_3,y_3,z_3] = calculate_XYZ(int(rect[0][0]), int(rect[0][0]),0,offset,[5,-30,-37])
            #label = data[i][0]+ ", Rotation Angle: " + str(angle) + ", degrees " + ", x= " +str(int(x_3))+ ", y= "+str(int(y_3))
            label= "calculated angle = " + str(actual_angle) + " actual angle = "+ str(angle)
            print(data)
            #textbox = cv.rectangle(image, (center[0]-150, center[1]+int(width/5)+160), 
            #(center[0] + 100, center[1]+int(width/5)+135), (255,255,255), -1)
            #cv.putText(image, label, (center[0]-150, center[1]+int(width/5)+150), 
            #cv.FONT_HERSHEY_SIMPLEX, 0.35, (0,0,0), 1, cv.LINE_AA)
            cv.drawContours(image,[box],0,(0,0,255),2)
            print(x_limit)
            print(y_limit)
            if data[i][0]!="logo":
                cv.line(image,(int(x_limit[350]),int(y_limit[350])),(int(x_limit[-350]),int(y_limit[-350])),(255,255,255), 2)
            #cv.circle(image,(int(box[0][0]),int(box[0][1])),3,(0,255,255),-1)
            #cv.circle(image,(int(box[3][0]),int(box[3][1])),3,(0,255,255),-1)
            #cv.circle(image,(int(box[1][0]),int(box[1][1])),3,(0,255,255),-1)
            #cv.circle(image,(int(box[2][0]),int(box[2][1])),3,(0,255,255),-1)
            #cv.circle(image,(int(rect[0][0])-10,int(rect[0][1])),8,(0,0,255),-1)
            #cv.line(image, (int(box[0][0]),int(box[0][1])), (int(box[0][0])+150,int(box[0][1])), (0,0,0), 1)
            #cv.putText(image,"0", (int(box[0][0])-2,int(box[0][1])-2),cv.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 3, cv.LINE_AA)
            #cv.putText(image,"1", (int(box[1][0])-2,int(box[1][1])-2),cv.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 3, cv.LINE_AA)
            #cv.putText(image,"3", (int(box[3][0])-2,int(box[3][1])-2),cv.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 3, cv.LINE_AA)
            #cv.putText(image,"2", (int(box[2][0])-2,int(box[2][1])-2),cv.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 3, cv.LINE_AA)
            cv.circle(image,(int((int(x_1)+int(x_2))/2),int((int(y_1)+int(y_2))/2)),5,(0,0,255),-1)
            cv.circle(image,(int(x_2),int(y_2)),5,(255,0,0),-1)
            cv.circle(image,(int(x_1),int(y_1)),5,(255,0,0),-1)
    logo_angle=0
    for k in range(no_of_objects):
        if data[k][0]=='logo':
            logo_angle=data[k][4]
    for k in range(no_of_objects):
        data[k][4]=data[k][4]-logo_angle
    cv.imshow("our calculations",image)
    cv.waitKey(0)
    cv.destroyAllWindows()
    return data
