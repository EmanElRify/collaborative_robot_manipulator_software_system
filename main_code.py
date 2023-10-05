import customtkinter as ctk
import tkinter as tk
from tkinter import ttk
from PIL import Image,ImageTk
import numpy as np
from Vision import *
from inverse_k_2D import *
import serial.tools.list_ports
import cv2



def set_background():
    global background_set
    _, frame = cap.read()
    global gray_image1
    gray_image1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Convert background image to PIL format
    global background_image
    background_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    # Resize the image to fit in the label
    background_image = background_image.resize((320, 240))
    # Convert the image to PhotoImage format
    background_image = ImageTk.PhotoImage(background_image)
    background_label.configure(image=background_image)
    background_label.image = background_image
    background_set = True

def start_processing():
    if not background_set:
        print("Please set the background first!")
        return

    def process_frames():
        while True:
            safety()
            global frame
            _, frame = cap.read()
            gray_image2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            Difference = np.absolute(np.matrix(np.int16(gray_image1)) - np.matrix(np.int16(gray_image2)))
            Difference[Difference > 255] = 255
            Difference = np.uint8(Difference)
            BW = Difference
            BW[BW <= 100] = 0
            BW[BW > 100] = 1
            column_sums = np.matrix(np.sum(BW, 0))
            column_numbers = np.matrix(np.arange(640))
            column_mult = np.multiply(column_sums, column_numbers)
            total = np.sum(column_mult)
            total_total = np.sum(np.sum(BW))
            column_location = total / total_total
            X_Location = column_location * cm_to_pixels
            row_sums = np.matrix(np.sum(BW, 1))
            row_sums = row_sums.transpose()
            row_numbers = np.matrix(np.arange(480))
            row_mult = np.multiply(row_sums, row_numbers)
            total = np.sum(row_mult)
            total_total = np.sum(np.sum(BW))
            row_location = total / total_total
            Y_Location = row_location * cm_to_pixels
            # x y z in camera coordinates
            PC = [[X_Location],
                  [Y_Location],
                  [0],
                  [1]]
            # x y z in base coordinates
            PO = np.dot(H0_C, PC)
            #print(PO)
            global XO
            global YO
            XO = PO[0]
            YO = PO[1]
            x_entry.delete(0,ctk.END)
            x_entry.insert(0,XO[0])
            y_entry.delete(0, ctk.END)
            y_entry.insert(0, YO[0])
            z_entry.delete(0, ctk.END)
            z_entry.insert(0,str(0))
            # IK_theta = calculate_ik_theta(XO, YO)  # Calculate IK theta
            # print(XO, YO, IK_theta)
            # Update x, y, and IK theta values in the GUI
            # x_label.config(text=f"X: {XO}")
            # y_label.config(text=f"Y: {YO}")
            # ik_label.config(text=f"IK Theta: {IK_theta}")

            # angle_set.delete(0, tk.END)
            # angle_set.insert(0, str(IK_theta[0]))

            if cv2.waitKey(5) == 27:
                break

    threading.Thread(target=process_frames, daemon=True).start()


def FeatureDetection():
    cap = cv2.VideoCapture(1)

    # Capture a frame
    ret, framei = cap.read()

    # Check if the frame was captured successfully
    if not ret:
        print("Error capturing frame")

    # Save the frame as an image file
    cv2.imwrite("screenshot.jpg", framei)

    # Close the camera
    cap.release()
    # Load the box image and the cluttered scene image
    global box_image

    box_image = cv2.imread('object.jpg')
    box_image_op=box_image
    box_image = Image.fromarray(cv2.cvtColor(box_image, cv2.COLOR_BGR2RGB))
    # Resize the image to fit in the label
    box_image = box_image.resize((320, 240))
    # Convert the image to PhotoImage format
    box_image = ImageTk.PhotoImage(box_image)
    box_label.configure(image=box_image)
    box_label.image = box_image

    global scene_image

    scene_image = cv2.imread('screenshot.jpg')
    scene_image_op=scene_image

    scene_image = Image.fromarray(cv2.cvtColor(scene_image, cv2.COLOR_BGR2RGB))
    # Resize the image to fit in the label
    scene_image = scene_image.resize((320, 240))
    # Convert the image to PhotoImage format
    scene_image = ImageTk.PhotoImage(scene_image)
    scene_label.configure(image=scene_image)
    scene_label.image = scene_image
    # Display the box and scene images
    # cv2.imshow('desired object Image', box_image)
    # cv2.imshow('Scene Image', scene_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # Convert the images to grayscale
    box_gray = cv2.cvtColor(box_image_op, cv2.COLOR_BGR2GRAY)
    scene_gray = cv2.cvtColor(scene_image_op, cv2.COLOR_BGR2GRAY)

    # Detect and extract SURF features from the box and scene images
    surf = cv2.SIFT_create(400)
    box_keypoints, box_descriptors = surf.detectAndCompute(box_gray, None)
    scene_keypoints, scene_descriptors = surf.detectAndCompute(scene_gray, None)

    # Match the features in the box image to those in the scene image
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
    matches = bf.match(box_descriptors, scene_descriptors)

    # Draw the matched features
    global match_image

    match_image = cv2.drawMatches(box_gray, box_keypoints, scene_gray, scene_keypoints, matches, None,
                                  flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    match_image_op=match_image
    match_image = Image.fromarray(cv2.cvtColor(match_image, cv2.COLOR_BGR2RGB))
    # Resize the image to fit in the label
    match_image = match_image.resize((320, 240))
    # Convert the image to PhotoImage format
    match_image = ImageTk.PhotoImage(match_image)
    match_label.configure(image=match_image)
    match_label.image = match_image
    # cv2.imshow('Matched Features', match_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # Estimate a geometric transformation between the box and the scene
    box_points = np.float32([box_keypoints[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    scene_points = np.float32([scene_keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
    transform, inlier_mask = cv2.estimateAffine2D(box_points, scene_points)

    # Draw the inlier points
    global inlier_image
    inlier_image = cv2.drawMatches(box_gray, box_keypoints, scene_gray, scene_keypoints, matches, None,
                                   matchColor=(0, 255, 0), matchesMask=inlier_mask.ravel().tolist(),
                                   flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    inlier_image = Image.fromarray(cv2.cvtColor(inlier_image, cv2.COLOR_BGR2RGB))
    # Resize the image to fit in the label
    inlier_image = inlier_image.resize((320, 240))
    # Convert the image to PhotoImage format
    inlier_image = ImageTk.PhotoImage(inlier_image)
    inlier_label.configure(image=inlier_image)
    inlier_label.image = inlier_image

    # cv2.imshow('Inlier Points', inlier_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # Draw a polygon around the detected box in the scene image
    box_polygon = np.float32(
        [[0, 0], [box_gray.shape[1], 0], [box_gray.shape[1], box_gray.shape[0]], [0, box_gray.shape[0]]]).reshape(-1, 1,
                                                                                                                  2)
    new_box_polygon = cv2.transform(box_polygon, transform)
    global scene_image_with_box
    scene_image_with_box = scene_image_op.copy()
    cv2.polylines(scene_image_with_box, [np.int32(new_box_polygon)], True, (0, 255, 255), thickness=3)

    # Compute the centroid of the polygon
    M = cv2.moments(np.int32(new_box_polygon))
    centroid_x = int(M['m10'] / M['m00'])
    centroid_y = int(M['m01'] / M['m00'])
    # Convert the centroid location from pixels to centimeters
    cm_to_pixels = 32.0 / 640.0
    centroid_x_cm = centroid_x * cm_to_pixels
    centroid_y_cm = centroid_y * cm_to_pixels

    # Draw the centroid
    cv2.circle(scene_image_with_box, (centroid_x, centroid_y), 5, (0, 0, 255), thickness=-1)

    scene_image_with_box = Image.fromarray(cv2.cvtColor(scene_image_with_box, cv2.COLOR_BGR2RGB))
    # Resize the image to fit in the label
    scene_image_with_box = scene_image_with_box.resize((320, 240))
    # Convert the image to PhotoImage format
    scene_image_with_box = ImageTk.PhotoImage(scene_image_with_box)
    scene_image_with_box_label.configure(image=scene_image_with_box)
    scene_image_with_box_label.image = scene_image_with_box
    # Display the detected box in the scene image
    # cv2.imshow('Detected Box', scene_image_with_box)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # Print the location of the centroid in centimeters
    print('Centroid Location: ({:.2f} cm, {:.2f} cm)'.format(centroid_x_cm, centroid_y_cm))
    # x y z in camera coordinates
    PC = [[centroid_x_cm],
          [centroid_y_cm],
          [0],
          [1]]
    # x y z in base coordinates
    PO = np.dot(H0_C, PC)
    global XO
    global YO
    XO = PO[0]
    YO = PO[1]
    x_entry2.delete(0, ctk.END)
    x_entry2.insert(0, XO[0])
    y_entry2.delete(0, ctk.END)
    y_entry2.insert(0, YO[0])
    z_entry2.delete(0, ctk.END)
    z_entry2.insert(0, str(0))


def safety():
    #get the webcam video stream
    #webcam_video_stream = cv2.VideoCapture(1,cv2.CAP_DSHOW)
    # Initialize the last prediction variable
    
    #create a while loop 
    #while True:
    #get the current frame from video stream
    ret,current_frame = cap.read()
    #use the video current frame instead of image
    img_to_detect = current_frame

    img_height = img_to_detect.shape[0]
    img_width = img_to_detect.shape[1]
    
    # convert to blob to pass into model
    img_blob = cv2.dnn.blobFromImage(img_to_detect, 0.003922, (320, 320), swapRB=True, crop=False)
    #recommended by yolo authors, scale factor is 0.003922=1/255, width,height of blob is 320,320
    #accepted sizes are 320×320,416×416,609×609. More size means more accuracy but less speed
    
    # set of 80 class labels 
    class_labels = ["person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
                    "trafficlight","firehydrant","stopsign","parkingmeter","bench","bird","cat",
                    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
                    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sportsball",
                    "kite","baseballbat","baseballglove","skateboard","surfboard","tennisracket",
                    "bottle","wineglass","cup","fork","knife","spoon","bowl","banana","apple",
                    "sandwich","orange","broccoli","carrot","hotdog","pizza","donut","cake","chair",
                    "sofa","pottedplant","bed","diningtable","toilet","tvmonitor","laptop","mouse",
                    "remote","keyboard","cellphone","microwave","oven","toaster","sink","refrigerator",
                    "book","clock","vase","scissors","teddybear","hairdrier","toothbrush"]
    
    #Declare List of colors as an array
    #Green, Blue, Red, cyan, yellow, purple
    #Split based on ',' and for every split, change type to int
    #convert that to a numpy array to apply color mask to the image numpy array
    class_colors = ["0,255,0","0,0,255","255,0,0","255,255,0","0,255,255"]
    class_colors = [np.array(every_color.split(",")).astype("int") for every_color in class_colors]
    class_colors = np.array(class_colors)
    class_colors = np.tile(class_colors,(16,1))
    
    # Loading pretrained model 
    # input preprocessed blob into model and pass through the model
    # obtain the detection predictions by the model using forward() method
    yolo_model = cv2.dnn.readNetFromDarknet('model/yolov3.cfg','model/yolov3.weights')
    
    # Get all layers from the yolo network
    # Loop and find the last layer (output layer) of the yolo network 
    yolo_layers = yolo_model.getLayerNames()
    yolo_output_layer = [yolo_layers[yolo_layer - 1] for yolo_layer in yolo_model.getUnconnectedOutLayers()]
    
    # input preprocessed blob into model and pass through the model
    yolo_model.setInput(img_blob)
    # obtain the detection layers by forwarding through till the output layer
    obj_detection_layers = yolo_model.forward(yolo_output_layer)
    
    
    ############## NMS Change 1 ###############
    # initialization for non-max suppression (NMS)
    # declare list for [class id], [box center, width & height[], [confidences]
    class_ids_list = []
    boxes_list = []
    confidences_list = []
    ############## NMS Change 1 END ###########
    
    
    # loop over each of the layer outputs
    for object_detection_layer in obj_detection_layers:
    	# loop over the detections
        for object_detection in object_detection_layer:
            
            # obj_detections[1 to 4] => will have the two center points, box width and box height
            # obj_detections[5] => will have scores for all objects within bounding box
            all_scores = object_detection[5:]
            predicted_class_id = np.argmax(all_scores)
            prediction_confidence = all_scores[predicted_class_id]
        
            # take only predictions with confidence more than 20%
            if prediction_confidence > 0.20:
                #get the predicted label
                predicted_class_label = class_labels[predicted_class_id]
                #obtain the bounding box co-oridnates for actual image from resized image size
                bounding_box = object_detection[0:4] * np.array([img_width, img_height, img_width, img_height])
                (box_center_x_pt, box_center_y_pt, box_width, box_height) = bounding_box.astype("int")
                start_x_pt = int(box_center_x_pt - (box_width / 2))
                start_y_pt = int(box_center_y_pt - (box_height / 2))
                
                ############## NMS Change 2 ###############
                #save class id, start x, y, width & height, confidences in a list for nms processing
                #make sure to pass confidence as float and width and height as integers
                class_ids_list.append(predicted_class_id)
                confidences_list.append(float(prediction_confidence))
                boxes_list.append([start_x_pt, start_y_pt, int(box_width), int(box_height)])
                ############## NMS Change 2 END ###########
                
    ############## NMS Change 3 ###############
    # Applying the NMS will return only the selected max value ids while suppressing the non maximum (weak) overlapping bounding boxes      
    # Non-Maxima Suppression confidence set as 0.5 & max_suppression threhold for NMS as 0.4 (adjust and try for better perfomance)
    max_value_ids = cv2.dnn.NMSBoxes(boxes_list, confidences_list, 0.5, 0.4)
    
    # loop through the final set of detections remaining after NMS and draw bounding box and write text
    for max_valueid in max_value_ids:
        max_class_id = max_valueid
        box = boxes_list[max_class_id]
        start_x_pt = box[0]
        start_y_pt = box[1]
        box_width = box[2]
        box_height = box[3]
        
        #get the predicted class id and label
        predicted_class_id = class_ids_list[max_class_id]
        predicted_class_label = class_labels[predicted_class_id]
        prediction_confidence = confidences_list[max_class_id]
    ############## NMS Change 3 END ########### 
               
        end_x_pt = start_x_pt + box_width
        end_y_pt = start_y_pt + box_height
        
        #get a random mask color from the numpy array of colors
        box_color = class_colors[predicted_class_id]
        
        #convert the color numpy array as a list and apply to text and box
        box_color = [int(c) for c in box_color]
        
        # print the prediction in console
        predicted_class_label = "{}: {:.2f}%".format(predicted_class_label, prediction_confidence * 100)
        print("predicted object {}".format(predicted_class_label))

        if predicted_class_label[0:6] =='person':
            serialInst.write('I'.encode('utf-8'))
            print('Person detected')
            
            # draw rectangle and text in the image
        cv2.rectangle(img_to_detect, (start_x_pt, start_y_pt), (end_x_pt, end_y_pt), box_color, 1)
        cv2.putText(img_to_detect, predicted_class_label, (start_x_pt, start_y_pt-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 1)

    #Displaylivefeedfrom camera
    frame = cv2.cvtColor(img_to_detect, cv2.COLOR_BGR2RGB)
    frame = Image.fromarray(frame)
    frame = frame.resize((320, 240))
    photo = ImageTk.PhotoImage(frame)
    live_feed_label.configure(image=photo)
    live_feed_label.image = photo

def forward_k():
    global end_effector, result
    end_effector = ctk.IntVar()
    theta1 = float(th1_e.get())
    theta2 = float(th2_e.get())
    # theta3 = float(th3_e.get())*(np.pi/180)
    # theta4 = float(th4_e.get())*(np.pi/180)
    result = np.array([[theta1],[theta2]])
    end_effector = calculate_forward_kinematics_robot(result)
    end_effector = np.transpose(end_effector)
  #  print(end_effector)
    robot_status.delete(0,ctk.END)
    robot_status.insert(0,str(end_effector))
    sendData()
    return end_effector

def exit_program():
    cap.release()
    cv2.destroyAllWindows()
    root.destroy()


def move_robot_f1():
    global x_inverse1, y_inverse1, z_inverse1, result
    x_inverse1 = x_e.get()
    x_inverse1 = float(x_inverse1)
    y_inverse1 = y_e.get()
    y_inverse1 = float(y_inverse1)
    z_inverse1 = z_e.get()
    z_inverse1 = float(z_inverse1)
    inverse_array = np.array([1, 0, 0, x_inverse1 * 0.01, y_inverse1 * 0.01, z_inverse1 * 0.01])

    # The inverse return an array of 4*1
    result = calculate_inverse_kinematics(inverse_array)
    robot_status.delete(0, ctk.END)
    robot_status.insert(0, str(result))
    sendData()

def move_robot_f2():
    global x_inverse2 , y_inverse2 , z_inverse2 ,result
    x_inverse2=x_entry.get()
    x_inverse2=float(x_inverse2)
    y_inverse2=y_entry.get()
    y_inverse2=float(y_inverse2)
    z_inverse2=z_entry.get()
    z_inverse2=float(z_inverse2)
    inverse_array = np.array([1,0,0,x_inverse2*0.01,y_inverse2*0.01,z_inverse2*0.01])
    
    # The inverse return an array of 4*1
    result = calculate_inverse_kinematics(inverse_array)
    sendData()

def move_robot_f3():
    global x_inverse3, y_inverse3, z_inverse3, result
    x_inverse3 = x_entry2.get()
    x_inverse3 = float(x_inverse3)
    y_inverse3 = y_entry2.get()
    y_inverse3 = float(y_inverse3)
    z_inverse3 = z_entry2.get()
    z_inverse3 = float(z_inverse3)
    inverse_array = np.array([1, 0, 0, x_inverse3 * 0.01, y_inverse3 * 0.01, z_inverse3 * 0.01])

    # The inverse return an array of 4*1
    result = calculate_inverse_kinematics(inverse_array)
    sendData()


def move_homing():
    #home position is used as the placing position
    #angles= home angle for each motor
    angles=[0,0]
    
    serialInst.write(angles[0].encode('utf-8'))
    serialInst.write(indexA.encode('utf-8'))

    serialInst.write(angles[1].encode('utf-8'))
    serialInst.write(indexB.encode('utf-8'))
    
    #serialInst.write(angles[2].encode('utf-8'))
    #serialInst.write(indexC.encode('utf-8'))

    #serialInst.write(angles[3].encode('utf-8'))
    #serialInst.write(indexD.encode('utf-8'))

    serialInst.write(newLine.encode('utf-8'))

    print("Data Sent")

def sendData():
    msg=''
    serialInst.write(str(float(result[0])).encode('utf-8'))
    msg=str(float(result[0])).encode('utf-8')
    serialInst.write(indexA.encode('utf-8'))
    msg=msg+indexA.encode('utf-8')
    serialInst.write(str(float(result[1])).encode('utf-8'))
    msg=msg+str(float(result[1])).encode('utf-8')
    serialInst.write(indexB.encode('utf-8'))
    msg=msg+indexB.encode('utf-8')
    serialInst.write(newLine.encode('utf-8'))
    msg=msg+newLine.encode('utf-8')
    print(msg)
    print("Data Sent")




# Defining the variables we'll use in our script
#motRun = "1" 
indexA =  "A"
indexB =  "B"
#indexC =  "C"
#indexD =  "D"

newLine = "\n"
#  Initialize the serial port
serialInst = serial.Serial()
portVal = './COM5'
print(f"Selecting port {portVal}")
serialInst.baudrate = 9600
serialInst.port = portVal
serialInst.open()



# MAIN Frame
root=ctk.CTk()
root.title("Cobot GUI")
root.geometry('700*700')

root.columnconfigure((0,1),weight=1)
root.rowconfigure((0,1),weight=1)

#Notebook
notebook=ttk.Notebook(root)

Tab_1=ttk.Frame(notebook)
Tab_2=ttk.Frame(notebook)
Tab_3=ttk.Frame(notebook)

# we dont pack the tabs by the usual way otherwise by this way
notebook.add(Tab_1,text='Main Program')
notebook.add(Tab_2,text='Vision GUI')
notebook.add(Tab_3,text='Featuring')
notebook.grid()

# MAIN PROGRAM
# Additional Inner Frames
IK_frame     =ctk.CTkFrame(Tab_1,width=300,height=200)
FK_frame     =ctk.CTkFrame(Tab_1,width=600,height=200)
display_frame=ctk.CTkFrame(Tab_1,width=300,height=200)
other_frame  =ctk.CTkFrame (Tab_1,width=600,height=200)

# grid of frames
IK_frame.grid_propagate(False)
IK_frame.grid(row=0,column=0,padx=5,pady=5,sticky='nsew')
FK_frame.grid_propagate(False)
FK_frame.grid(row=0,column=1,padx=5,pady=5,sticky='nsew')
display_frame.grid_propagate(False)
display_frame.grid(row=1,column=0,padx=5,pady=5,sticky='nsew')
other_frame.grid_propagate(False)
other_frame.grid(row=1,column=1,padx=5,pady=5,sticky='nsew')

# Widgets Definition Section

# IK frame Widgets
IK_frame.columnconfigure((0,1,2),weight=1)
IK_frame.rowconfigure((0,1,2),weight=1)

x_e     =ctk.CTkEntry(IK_frame,width=50)
y_e     =ctk.CTkEntry(IK_frame,width=50)
z_e     =ctk.CTkEntry(IK_frame,width=50)

x_label =ctk.CTkLabel(IK_frame,text="X",font=('Roboto',30))
y_label =ctk.CTkLabel(IK_frame,text="Y",font=('Roboto',30))
z_label =ctk.CTkLabel(IK_frame,text="Z",font=('Roboto',30))
Move_button1=ctk.CTkButton(IK_frame,text="Move",font=('Roboto',30),command=move_robot_f1)
# IK grid
x_e.grid(row=0,column=0,padx=5,pady=5)
y_e.grid(row=0,column=1,padx=5,pady=5)
z_e.grid(row=0,column=2,padx=5,pady=5)

x_label.grid(row=1,column=0,padx=5,pady=5)
y_label.grid(row=1,column=1,padx=5,pady=5)
z_label.grid(row=1,column=2,padx=5,pady=5)

Move_button1.grid(row=2,column=0,columnspan=3,pady=5)

# FK Frame Widgets
FK_frame.columnconfigure((0,1,2,3,4),weight=1)
FK_frame.rowconfigure((0,1,2,3),weight=1)
slider_var1 = ctk.IntVar()
slider_var2 = ctk.IntVar()
# slider_var3 = ctk.IntVar()
# slider_var4 = ctk.IntVar()
# slider_var5 = ctk.IntVar()
slider_var6 = ctk.IntVar()

th1_e=ctk.CTkEntry(FK_frame,width=50,textvariable=slider_var1)
th2_e=ctk.CTkEntry(FK_frame,width=50,textvariable=slider_var2)
# th3_e=ctk.CTkEntry(FK_frame,width=50,textvariable=slider_var3)
# th4_e=ctk.CTkEntry(FK_frame,width=50,textvariable=slider_var4)
#th5_e=ctk.CTkEntry(FK_frame,width=50,textvariable=slider_var5)
grr_e=ctk.CTkEntry(FK_frame,width=50,textvariable=slider_var6)

th1_slider=ctk.CTkSlider(FK_frame,from_=0,to=180,variable=slider_var1)
th2_slider=ctk.CTkSlider(FK_frame,from_=0,to=180,variable=slider_var2)
# th3_slider=ctk.CTkSlider(FK_frame,from_=0,to=180,variable=slider_var3)
# th4_slider=ctk.CTkSlider(FK_frame,from_=0,to=180,variable=slider_var4)
#th5_slider=ctk.CTkSlider(FK_frame,from_=0,to=180,variable=slider_var5)
grr_slider=ctk.CTkSlider(FK_frame,from_=0,to=180,variable=slider_var6)

th1_label=ctk.CTkLabel(FK_frame,text='Theta 1',font=('Roboto',20))
th2_label=ctk.CTkLabel(FK_frame,text='Theta 2',font=('Roboto',20))
# th3_label=ctk.CTkLabel(FK_frame,text='Theta 3',font=('Roboto',20))
# th4_label=ctk.CTkLabel(FK_frame,text='Theta 4',font=('Roboto',20))
#th5_label=ctk.CTkLabel(FK_frame,text='Theta 5',font=('Roboto',20))
grr_label=ctk.CTkLabel(FK_frame,text='gripper',font=('Roboto',20))

Move_button2=ctk.CTkButton(FK_frame,text='Move',font=('Roboto',30),command=forward_k)
#################### FK Grid #######################################
th1_e.grid(row=0,column=0,padx=5,pady=5)
th2_e.grid(row=0,column=1,padx=5,pady=5)
# th3_e.grid(row=0,column=2,padx=5,pady=5)
# th4_e.grid(row=0,column=3,padx=5,pady=5)
#th5_e.grid(row=0,column=4,padx=5,pady=5)
grr_e.grid(row=0,column=4,padx=5,pady=5)

th1_slider.grid(row=1,column=0,padx=5,pady=5)
th2_slider.grid(row=1,column=1,padx=5,pady=5)
# th3_slider.grid(row=1,column=2,padx=5,pady=5)
# th4_slider.grid(row=1,column=3,padx=5,pady=5)
#th5_slider.grid(row=1,column=4,padx=5,pady=5)
grr_slider.grid(row=1,column=4,padx=5,pady=5)

th1_label.grid(row=2,column=0,padx=5,pady=5)
th2_label.grid(row=2,column=1,padx=5,pady=5)
# th3_label.grid(row=2,column=2,padx=5,pady=5)
# th4_label.grid(row=2,column=3,padx=5,pady=5)
##th5_label.grid(row=2,column=4,padx=5,pady=5)
grr_label.grid(row=2,column=4,padx=5,pady=5)

Move_button2.grid(row=3,column=0,columnspan=6,pady=5)
################################ DISPLAY FRAME Widgets ############################################
display_frame.columnconfigure((0,1),weight=1)
display_frame.rowconfigure((0,1),weight=1)

robot_status=ctk.CTkEntry(display_frame)
robot_status_label=ctk.CTkLabel(display_frame,text="Robot Status Display",font=('Roboto',15))
# save_location=ctk.CTkButton(display_frame,text="Save This Location",font=('Roboto',15))
# display frame grid
robot_status.grid(row=0,column=0,padx=5,pady=5,sticky='nsew')
robot_status_label.grid(row=1,column=0,padx=5,pady=5,sticky='nsew')
# save_location.grid(row=0,rowspan=2,column=1,padx=5,pady=5)

########################### Other Frame #################################################
other_frame.columnconfigure((0),weight=1)
other_frame.rowconfigure((0,1,2),weight=1)

start=ctk.CTkButton(other_frame,text="Light Mode",font=('Roboto',30),command=lambda: ctk.set_appearance_mode('light'))
# stop=ctk.CTkButton(other_frame,text="Stop",font=('Roboto',30))
exit=ctk.CTkButton(other_frame,text="Exit",command=exit_program,font=('Roboto',30))
reset=ctk.CTkButton(other_frame,text="Dark Mode",command=lambda: ctk.set_appearance_mode('dark') ,font=('Roboto',30))
# Other Frame grid

start.grid(row=0,column=0,padx=5,pady=5,sticky='nsew')
#stop.grid(row=0,column=1,padx=5,pady=5,sticky='nsew')
exit.grid(row=2,column=0,padx=5,pady=5,sticky='nsew')
reset.grid(row=1,column=0,padx=5,pady=5,sticky='nsew')


############## Second Window #####################################
Tab_2.columnconfigure((0,1),weight=1)
Tab_2.rowconfigure(0,weight=4)
Tab_2.rowconfigure(1,weight=1)


background_frame=ctk.CTkFrame(Tab_2,width=50,height=100)
processing_frame=ctk.CTkFrame(Tab_2,width=50,height=100)
obj_loc_frame   =ctk.CTkFrame(Tab_2,width=50,height=50)
btns_frame      =ctk.CTkFrame(Tab_2,width=50,height=50)



background_frame.grid_propagate(False)
background_frame.grid(row=0,column=0,padx=5,pady=5,sticky='nsew')
processing_frame.grid(row=0,column=1,padx=5,pady=5,sticky='nsew')
obj_loc_frame.grid(row=1,column=0,rowspan=2,padx=5,pady=5,sticky='nsew')
btns_frame.grid(row=1,column=1,rowspan=2,padx=5,pady=5,sticky='nsew')

######################## Background Frame ################################################
background_frame.grid_columnconfigure(0,weight=1)
background_frame.grid_rowconfigure((0),weight=3)
background_frame.grid_rowconfigure((1),weight=1)

background_label = ttk.Label(background_frame)
bk_button        = ctk.CTkButton(background_frame,text='Detect Workspace',font=('Roboto',30),command=set_background)

background_label.grid(row=0,column=0,padx=10,pady=10,sticky='nsew')
bk_button.grid(row=1,column=0,padx=5,pady=5,sticky='nsew')
###################### Processing Frame ################################
processing_frame.grid_columnconfigure(0,weight=1)
processing_frame.grid_rowconfigure((0),weight=3)
processing_frame.grid_rowconfigure((1),weight=1)


live_feed_label = ttk.Label(processing_frame)
procs_label=ctk.CTkLabel(processing_frame,text='Live Feed from Camera',font=('Roboto',30))

live_feed_label.grid(row=0, column=0, padx=10, pady=10,sticky='nsew')
procs_label.grid(row=1,column=0,padx=5,pady=5,sticky='nsew')
######################### Object Location Frame #########################################
obj_loc_frame.columnconfigure((0,1,2),weight=1)
obj_loc_frame.rowconfigure((0,1,2),weight=1)

x_entry=ctk.CTkEntry(obj_loc_frame,width=100)
y_entry=ctk.CTkEntry(obj_loc_frame,width=100)
z_entry=ctk.CTkEntry(obj_loc_frame,width=100)
x_label2=ctk.CTkLabel(obj_loc_frame,text="X",font=('Roboto',30))
y_label2=ctk.CTkLabel(obj_loc_frame,text="Y",font=('Roboto',30))
z_label2=ctk.CTkLabel(obj_loc_frame,text="Z",font=('Roboto',30))

obj_label=ctk.CTkLabel(obj_loc_frame,text='Object Location',font=('Arial',30),text_color='black')
#my_image =ImageTk.PhotoImage(Image.open(r"img.PNG"),size=(200,200))
#image_label=ttk.Label(background_frame,image=my_image)
x_entry.grid(row=0,column=0,padx=5,pady=5)
y_entry.grid(row=0,column=1,padx=5,pady=5)
z_entry.grid(row=0,column=2,padx=5,pady=5)

x_label2.grid(row=1,column=0,padx=5,pady=5)
y_label2.grid(row=1,column=1,padx=5,pady=5)
z_label2.grid(row=1,column=2,padx=5,pady=5)
obj_label.grid(row=2,column=0,columnspan=3,padx=5,pady=5,sticky='nsew')
#image_label.grid(row=1,column=0,padx=5,pady=5)

btns_frame.columnconfigure(0,weight=1)
btns_frame.rowconfigure((0,1,2),weight=1)
start2     = ctk.CTkButton(btns_frame,text="Start",command=start_processing)
exit2      = ctk.CTkButton(btns_frame,text="Exit",command=exit_program)
move_robot = ctk.CTkButton(btns_frame,text="Move Robot",command=move_robot_f2)

start2.grid(row=0,column=0,padx=5,pady=5,sticky='nsew')
exit2.grid(row=1,column=0,padx=5,pady=5,sticky='nsew')
move_robot.grid(row=2,column=0,padx=5,pady=5,sticky='nsew')

######################## Third Tab Frames  ####################################################
Tab_3.columnconfigure((0,1),weight=1)
Tab_3.rowconfigure(0,weight=4)
Tab_3.rowconfigure(1,weight=1)

feature_frame     =ctk.CTkFrame(Tab_3,width=300,height=300)
workspace_frame   =ctk.CTkFrame(Tab_3,width=300,height=300)
location_frame    =ctk.CTkFrame(Tab_3,width=300,height=300)
homing_frame      =ctk.CTkFrame(Tab_3,width=100,height=100)

# feature_frame.grid_propagate(False)

feature_frame.grid(row=1,column=1,rowspan=2,padx=5,pady=5,sticky='nsew')
workspace_frame.grid(row=0,column=1,padx=5,pady=5,sticky='nsew')
location_frame.grid(row=1,column=0,rowspan=2,padx=5,pady=5,sticky='nsew')
homing_frame.grid(row=0,column=0,padx=5,pady=5,sticky='nsew')

################## Feature Frame ############################################
feature_frame.grid_columnconfigure((0,1),weight=1)
feature_frame.grid_rowconfigure((0),weight=3)
feature_frame.grid_rowconfigure((1,2),weight=1)

match_label      = ttk.Label(feature_frame)
inlier_label     = ttk.Label(feature_frame)

feature_button_1  = ctk.CTkLabel(feature_frame,text='Matched Features',font=('Roboto',20))
feature_button_2  = ctk.CTkLabel(feature_frame,text='Inlayer Points',font=('Roboto',20))
feature_button_3  = ctk.CTkLabel(feature_frame,text='Feature Based Object Detection',font=('Roboto',20))

match_label.grid(row=0,column=0,padx=10,pady=10,sticky='nsew')
inlier_label.grid(row=0,column=1,padx=10,pady=10,sticky='nsew')
feature_button_1.grid(row=1,column=0,padx=5,pady=5,sticky='nsew')
feature_button_2.grid(row=1,column=1,padx=5,pady=5,sticky='nsew')
feature_button_3.grid(row=2,column=0,padx=5,pady=5,sticky='nsew',columnspan=2)

############################# Workspace Frame  ###########################################
workspace_frame.grid_columnconfigure(0,weight=3)
workspace_frame.grid_columnconfigure(1,weight=1)
workspace_frame.grid_rowconfigure((0),weight=3)
workspace_frame.grid_rowconfigure((1),weight=1)


box_label  = ttk.Label(workspace_frame)
scene_label  = ttk.Label(workspace_frame)
procs1_label= ctk.CTkLabel(workspace_frame,text='object to be detected',font=('Roboto',30))
procs2_label= ctk.CTkLabel(workspace_frame,text='workspace',font=('Roboto',30))

box_label.grid(row=0, column=0, padx=10, pady=10,sticky='nsew')
scene_label.grid(row=0, column=1, padx=10, pady=10,sticky='nsew')
procs1_label.grid(row=1,column=0,padx=5,pady=5,sticky='nsew')
procs2_label.grid(row=1,column=1,padx=5,pady=5,sticky='nsew')

####################### Location frame #######################################
obj_loc_frame.columnconfigure(0,weight=3)
obj_loc_frame.columnconfigure((1,2),weight=1)
obj_loc_frame.rowconfigure(0,weight=3)
obj_loc_frame.rowconfigure((1,2,3),weight=1)


scene_image_with_box_label = ttk.Label(location_frame)
x_entry2=ctk.CTkEntry(location_frame,width=100)
y_entry2=ctk.CTkEntry(location_frame,width=100)
z_entry2=ctk.CTkEntry(location_frame,width=100)
x_label23=ctk.CTkLabel(location_frame,text="X",font=('Roboto',30))
y_label23=ctk.CTkLabel(location_frame,text="Y",font=('Roboto',30))
z_label23=ctk.CTkLabel(location_frame,text="Z",font=('Roboto',30))

location_label=ctk.CTkLabel(location_frame,text='Object Location',font=('Arial',30),text_color='black')
#my_image =ImageTk.PhotoImage(Image.open(r"img.PNG"),size=(200,200))
#image_label=ttk.Label(background_frame,image=my_image)
scene_image_with_box_label.grid(row=0,column=0,columnspan=3,padx=5,pady=5,sticky='nsew')
x_entry2.grid(row=1,column=0,padx=5,pady=5,sticky='nsew')
y_entry2.grid(row=1,column=1,padx=5,pady=5,sticky='nsew')
z_entry2.grid(row=1,column=2,padx=5,pady=5,sticky='nsew')

x_label23.grid(row=2,column=0,padx=5,pady=5,sticky='nsew')
y_label23.grid(row=2,column=1,padx=5,pady=5,sticky='nsew')
z_label23.grid(row=2,column=2,padx=5,pady=5,sticky='nsew')
location_label.grid(row=3,column=0,columnspan=3,padx=5,pady=5,sticky='nsew')
#image_label.grid(row=1,column=0,padx=5,pady=5)


################## Homing frame ###############################

homing_frame.columnconfigure(0,weight=1)
homing_frame.rowconfigure((0,1,2,3),weight=1)
start3        = ctk.CTkButton(homing_frame,text="Start_Detection",command=FeatureDetection)
exit3         = ctk.CTkButton(homing_frame,text="Exit",command=exit_program)
move_robot2   = ctk.CTkButton(homing_frame,text="Move Robot",command=move_robot_f3)
# homing_button = ctk.CTkButton(homing_frame,text="Hoiming")

start3.grid(row=0,column=0,padx=5,pady=5,sticky='nsew')
exit3.grid(row=1,column=0,padx=5,pady=5,sticky='nsew')
move_robot2.grid(row=2,column=0,padx=5,pady=5,sticky='nsew')
# homing_button.grid(row=3,column=0,padx=5,pady=5,sticky='nsew')


root.mainloop()