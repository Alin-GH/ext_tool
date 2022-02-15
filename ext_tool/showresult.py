'''
show data, draw 3D/2D bboxes to image.

'''

import numpy as np 
import os
import math
import glob
import cv2

#### prepared ground truth be like 
class GT_Info(object):
    def __init__(self, line):
        self.name = line[0]

        # x, y, z in object coordinate, meter
        self.x = float(line[1])
        self.y = float(line[2])
        self.z = float(line[3])

        # length, weight, height in object coordinate, meter
        self.l = float(line[4])
        self.w = float(line[5])
        self.h = float(line[6])

 
        # object orientation [-pi, pi]
        self.rot_z = float(line[7])

        # in pixel coordinate
        self.xmin = float(line[8])
        self.ymin = float(line[9])
        self.xmax = float(line[10])
        self.ymax = float(line[11])

### R_t matrix
def get_extrensic_matrix(params: dict):
    """
    Return the extrensic matrix for converting 
    from object coordinates 
    to camera  coordinates. 
    """
    from math import sin as s
    from math import cos as c
    from math import radians
    y = radians(params["yaw"])   # yaw       atZ     [radians]
    p = radians(params["pitch"]) # pitch     atY     [radians]
    r = radians(params["roll"])  # roll      atX     [radians]
    x_v = params["x_t"]        # camera location at object coordinate x [meters]
    y_v = params["y_t"]        # ---------- y [meters]
    z_v = params["z_t"]        # ---------- z [meters]


    #####   modified
    R_v2c = np.array([[c(y)*c(p),  -s(y)*c(r)+c(y)*s(p)*s(r) ,  s(y)*s(r)+c(y)*s(p)*c(r) ],
                     [ s(y)*c(p),   c(y)*c(r)+s(y)*s(p)*s(r) ,  -c(y)*s(r)+s(y)*s(p)*c(r)],
                     [-s(p)     ,        c(p)*s(r)           ,  c(p)*c(r)                ]])


    # Translation from velodyne
    t_v2c = np.array([[x_v], [y_v], [z_v]])

     
    t_v2c = -np.dot(R_v2c, t_v2c) 
    r_t = np.hstack((R_v2c, t_v2c))
    ones = [0,0,0,1]
    Rt_v2c = np.vstack((r_t, ones))

    return Rt_v2c

### intrinsic matrix
def get_intrensic_matrix(camera_params: dict):
    """
    intrensic matrix for converting from camera coordinates to image. 
    """
    fx = camera_params["fx"]                      # fx
    fy = camera_params["fy"]                      # fy
    cx = camera_params["x_center"]                # px    ## Center of the image in x
    cy = camera_params["y_center"]                # px    ## Center of the image in y

    # K original
    K_o = np.array([[fx,   0,    cx],
                    [0,    fy,   cy],
                    [0,    0,    1]])
    
    # Matrix  
    cali = np.zeros((3,1), int)
    K_m = np.hstack((K_o, cali))
    
    return K_m

### 8 points at camera coordinates
def compute_8_points_c(objs, R_t_params, intr_params):

    if isinstance(objs, list):
        obj = GT_Info(objs)
    else:
        print('obj does not support')

    # 3D Bounding Box with angle 
    R_z = np.array([[np.cos(obj.rot_z),  -np.sin(obj.rot_z), 0],
                    [np.sin(obj.rot_z),   np.cos(obj.rot_z), 0],
                    [0, 0, 1]])

    x_corners = [0, obj.l, obj.l, obj.l, obj.l, 0,     0,      0    ]  # -l/2
    y_corners = [0, 0,     obj.w, obj.w, 0,     0,     obj.w,  obj.w]  # -w/2
    z_corners = [0, 0,     0,     obj.h, obj.h, obj.h, obj.h,  0    ]  # -h/2

    x_corners = [i - obj.l/2 for i in x_corners]
    y_corners = [i - obj.w/2 for i in y_corners]
    z_corners = [i - obj.h/2 for i in z_corners]

    corners_3D = np.array([x_corners, y_corners, z_corners])
    corners_3D = R_z.dot(corners_3D)
    corners_3D += np.array([obj.x, obj.y, obj.z]).reshape((3, 1))

    ### 4*8
    corners_3D_1 = np.vstack((corners_3D, np.ones((corners_3D.shape[-1]))))
    # print(corners_3D_1)

    ### 4*4
    R_t = get_extrensic_matrix(R_t_params) 

    corners_3D_c = R_t.dot(corners_3D_1)
    # print(corners_3D_c)
    
    ### to view in image
    intr_m = get_intrensic_matrix(intr_params)
    # print(intr_m)
    prev_p = intr_m.dot(corners_3D_c)

    # print("-------------")
    prev_p_n = prev_p / prev_p[2]
    # print(prev_p_n[:2])
    res = prev_p_n[:2]
    return res    

### minmax 8 to 4
def get_rec(bbx_3D):
    ## bbox_3D.shape: (2,8) 
    xmin = min(bbx_3D[0])
    ymin = min(bbx_3D[1])
    xmax = max(bbx_3D[0])
    ymax = max(bbx_3D[1])

    mins = int(xmin), int(ymin)
    maxs = int(xmax), int(ymax)
    # recs = mins, maxs

    return mins, maxs

###  to view
def draw_2Dbbox(img, obj_list, R_t, intr):

    for obj in obj_list:
        objr = obj[:12]
        corners_8 = compute_8_points_c(objr, R_t, intr)
        lr,rb = get_rec(corners_8)
        cv2.rectangle(img, lr, rb, (0,255,0), 1)

### show result 
def showresult(allpath, R_t, intr):
    img_dir = allpath + '/image_2/'
    label_dir = allpath + '/label_2/'    
      
    #getting images
    try:
        imgfiles = [x for x in sorted(glob.glob(img_dir+'/*.jpg'))]
    except:
        print("\nError: There are no images")
        exit()

    for imfile in imgfiles:
        #read images 
        im = cv2.imread(imfile)
        name = imfile.split('.')[0]
        name = name.split('/')[-1]

        label_file = label_dir + name + '.txt'
        if not os.path.exists(label_file):
            continue

        #getting labels
        obj_lists = [i.split(' ') for i in open(label_file).readlines()]
        
        #draw
        draw_2Dbbox(im, obj_lists, R_t, intr)    
        
        # cv2.imshow('res', im)

        yaw = R_t['yaw']
        pitch = R_t['pitch']
        roll = R_t['roll']
        
        origin_x_w_c = R_t['x_t']
        origin_y_w_c = R_t['y_t']
        origin_z_w_c = R_t['z_t']

        cv2.imshow('res', im) 

        while 1:
            
            key = cv2.waitKey(0)
            if key == ord('q'):
                text_show = "%f,%f,%f,%f,%f,%f\n"%(pitch, yaw, roll, origin_x_w_c, origin_y_w_c, origin_z_w_c)
                print( name + " " + text_show)

                #write label txt
                file_lb=open('exs/{}.txt'.format(name),'w')

                file_lb.write(str(text_show)+ ' ')

                file_lb.close()

                break

            elif key == ord('a'):
                break
            elif key == ord('e'):
                pitch += 1
                rt_params.update({'pitch': pitch})
                draw_2Dbbox(im, obj_lists, R_t, intr)
                cv2.imshow('new', im)
            elif key == ord('r'):
                pitch -= 1
                rt_params.update({'pitch': pitch})
                draw_2Dbbox(im, obj_lists, R_t, intr)
                cv2.imshow('new', im)
            
            elif key == ord('m'):
                yaw += 1
                rt_params.update({'yaw': yaw})
                draw_2Dbbox(im, obj_lists, R_t, intr)
                cv2.imshow('new', im)
            elif key == ord('n'):
                yaw -= 1
                rt_params.update({'yaw': yaw})
                draw_2Dbbox(im, obj_lists, R_t, intr)
                cv2.imshow('new', im)
            elif key == ord('o'):
                roll += 1
                rt_params.update({'roll': roll})
                draw_2Dbbox(im, obj_lists, R_t, intr)
                cv2.imshow('new', im)

            elif key == ord('p'):
                roll -= 1
                rt_params.update({'roll': roll})
                draw_2Dbbox(im, obj_lists, R_t, intr)
                cv2.imshow('new', im)
            
            elif key == ord('j'):
                origin_x_w_c -= 0.1

            elif key == ord('l'):
                origin_x_w_c += 0.1

            elif key == ord('i'):
                origin_y_w_c -= 0.1

            elif key == ord('k'):
                origin_y_w_c += 0.1

            elif key == ord('z'):
                origin_z_w_c -= 0.1
            elif key == ord('x'):
                origin_z_w_c += 0.1


            if cv2.waitKey(0) == 32: #space bar
                exit()

if __name__ == '__main__':
    ### extr
    '''
    camera0:topic: " "
    child_id: "camera60_front"
    frame_id: "smartcar"
    yaw: 89.46
    pitch: 179.84
    roll: 81.74
    x: 5.13
    y: 0.02
    z: 2.61  
    '''

    ### modified 
    '''
    yaw: 86.2217  pitch: -98.279   roll: 3.5787
    x: 0.0746685  y: 3.32017  z: -4.6971
    '''


    #### intr
    '''
    camera_matrix:
    rows: 3
    cols: 3
    data: [1375.58851,    0.     ,  658.21745,
            0.     , 1376.01224,  366.69729,
            0.     ,    0.     ,    1.     ]
    '''
    data_dir = 'json_out'
    
    ### MODIFIED 
    rt_params = dict(yaw  = 86.2217,         
                    pitch = -98.279,         
                    roll  = 3.5787,       
                    x_t   = 5.13,            
                    y_t   = 0.02,             
                    z_t   = 2.61)          


    intr_params = dict( fx = 1375.58851,            
                        fy = 1376.01224,            
                        x_center = 658.21745,           
                        y_center = 366.69729 )

    showresult(data_dir, rt_params, intr_params)
