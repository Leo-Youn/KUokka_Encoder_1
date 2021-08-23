import cv2, dlib
import numpy as np
import sys, os

# direction and location
direction = 0   # 얼굴 방향 (왼쪽, 오른쪽)
location = 0    # 얼굴 위치 (위쪽, 아래쪽)

# resize scaler
scaler = 1

# initialize face detector and shape predictor
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor('shape_predictor_68_face_landmarks.dat')

# load video
cap = cv2.VideoCapture(0)

face_roi = []
face_sizes = []

# loop
while True:
  # read frame buffer from video
  ret, img = cap.read()
  if not ret:
    break

  # resize frame
  img = cv2.resize(img, (int(img.shape[1] * scaler), int(img.shape[0] * scaler)))
  ori = img.copy()

  # find faces
  if len(face_roi) == 0:
    faces = detector(img, 1)
  else:
    roi_img = img[face_roi[0]:face_roi[1], face_roi[2]:face_roi[3]]
    # cv2.imshow('roi', roi_img)
    faces = detector(roi_img)

  # no faces
  if len(faces) == 0:
    print('no faces!')

    # if angle over 45
    if direction == 1 :    # 만약 얼굴이 왼쪽으로
          direction = 3    # 45도 이상 넘어간 경우
    elif direction == 2 :  # 만약 얼굴이 오른쪽으로
          direction = 4    # 45도 이상 넘어간 경우
    elif direction == 0 :  # 시작부터 얼굴을 찾을 수 없을 때
          direction = 5    # 얼굴을 못찾는 경우

    # if face over screen
    if location == 1 :    # 만약 얼굴이 위쪽으로
          location = 3    # 넘어간 경우
    elif location == 2 :  # 만약 얼굴이 아래쪽으로
          location = 4    # 넘어간 경우
    elif location == 0 :  # 시작부터 얼굴을 찾을 수 없을 때
          location = 5    # 얼굴을 못찾는 경우

  # find facial landmarks
  for face in faces:
    if len(face_roi) == 0:
      dlib_shape = predictor(img, face)
      shape_2d = np.array([[p.x, p.y] for p in dlib_shape.parts()])
    else:
      dlib_shape = predictor(roi_img, face)
      shape_2d = np.array([[p.x + face_roi[2], p.y + face_roi[0]] for p in dlib_shape.parts()])

    for s in shape_2d:
      cv2.circle(img, center=tuple(s), radius=1, color=(255, 255, 255), thickness=2, lineType=cv2.LINE_AA)
    
    # compute face center
    center_x, center_y = np.mean(shape_2d, axis=0).astype(np.int)

    # compute face boundaries
    min_coords = np.min(shape_2d, axis=0)
    max_coords = np.max(shape_2d, axis=0)

    # draw min, max coords
    cv2.circle(img, center=tuple(min_coords), radius=1, color=(255, 0, 0), thickness=2, lineType=cv2.LINE_AA)
    cv2.circle(img, center=tuple(max_coords), radius=1, color=(255, 0, 0), thickness=2, lineType=cv2.LINE_AA)

    # compute face size
    face_size = max(max_coords - min_coords)
    face_sizes.append(face_size)
    if len(face_sizes) > 10:
      del face_sizes[0]
    mean_face_size = int(np.mean(face_sizes) * 1.8)

    # compute face roi
    face_roi = np.array([int(min_coords[1] - face_size / 2), int(max_coords[1] + face_size / 2), int(min_coords[0] - face_size / 2), int(max_coords[0] + face_size / 2)])
    face_roi = np.clip(face_roi, 0, 10000)


    # ------------------------------------------------------------------
    # ----------------------------face angle----------------------------
    # ------------------------------------------------------------------
  
    # eyes
    left_eye = [int((shape_2d[43][0]+shape_2d[44][0]+shape_2d[46][0]+shape_2d[47][0])/4), int((shape_2d[43][1]+shape_2d[44][1]+shape_2d[46][1]+shape_2d[47][1])/4)]
    right_eye = [int((shape_2d[37][0]+shape_2d[38][0]+shape_2d[40][0]+shape_2d[41][0])/4), int((shape_2d[37][1]+shape_2d[38][1]+shape_2d[40][1]+shape_2d[41][1])/4)]
    cv2.circle(img, center=tuple(left_eye), radius=1, color=(0, 0, 255), thickness=2, lineType=cv2.LINE_AA)
    cv2.circle(img, center=tuple(right_eye), radius=1, color=(0, 0, 255), thickness=2, lineType=cv2.LINE_AA)

    # nose
    nose = shape_2d[30]
    cv2.circle(img, center=tuple(nose), radius=1, color=(0, 0, 255), thickness=2, lineType=cv2.LINE_AA)
    
    # direction - left or right
    # 0 - center, 1 - left, 2 - right
    if (left_eye[0] - nose[0]) > (nose[0] - right_eye[0]):
      if (left_eye[0] - nose[0]) - (nose[0] - right_eye[0]) > 20:
        direction = 2   # right
      else:
        direction = 0   # center
    else:
      if (nose[0] - right_eye[0] - (left_eye[0] - nose[0])) > 20:
        direction = 1   # left   
      else:
        direction = 0   # center

    # center_screen, center_face
    height, width, channel = img.shape
    center_screen = [int(width/2), int(height/2)]
    center = [center_x, center_y]
    cv2.circle(img, center=tuple(center_screen), radius=1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    cv2.circle(img, center=tuple(center), radius=1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

    # location - top or bottom
    # 0 - center, 1 - top, 2 - bottom
    if center[1] > height/2:
      if center[1] - height/2 > 20:
          location = 2   # bottom
      else:
          location = 0   # center
    else:
        if height/2 - center[1] > 20:
          location = 1   # top
        else:
          location = 0   # center 

    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
  

  # ------------------------------------------------------------------
  # ------------------------------result------------------------------
  # ------------------------------------------------------------------
  
  # 각 경우에 맞게 모터 제어 필요
  os.system('cls')

  # direction
  if direction == 0 :
        print("direction : center")
  elif direction == 1 :
        print("direction : left") 
  elif direction == 2 :
        print("direction : right")
  elif direction == 3 :
        print("direction : left over")
  elif direction == 4 :
        print("direction : right over")
  elif direction == 5 :
        print("direction : no face")
    
  # location
  if location == 0 :
        print("location : center")
  elif location == 1 :
        print("location : top")  
  elif location == 2 :
        print("location : bottom") 
  elif location == 3 :
        print("location : top over")
  elif location == 4 :
        print("location : bottom over")
  elif location == 5 :
        print("location : no face")

  # ------------------------------------------------------------------
  # ------------------------------------------------------------------
  # ------------------------------------------------------------------

  # visualize
  #cv2.imshow('original', ori)
  img = cv2.flip(img, 1)
  cv2.imshow('facial landmarks', img)

  if cv2.waitKey(1) == ord('q'):
    sys.exit(1)