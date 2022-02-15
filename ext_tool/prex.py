import os
import json
import math
import glob


def get_label(category):
    if category == "\u8d27\u8f66":
        return 'truck'
    elif category == "\u5c0f\u6c7d\u8f66":
        return 'car'
    elif category == "\u5df4\u58eb":
        return 'van'
    else :
        return 'ignore'  

def extract_json(json_path=str):
    with open(json_path) as f:
        json_data = json.load(f)
        img_obj_list = json_data['frames'][0]['images'][0]['items']
        lidar_obj_list = json_data['frames'][0]['items']           
        filename = json_data['frames'][0]['frameUrl'].split('/')[-1].split('.')[0]
        objs = []
        for img_obj in img_obj_list:
            for  lidar_obj in  lidar_obj_list:
                if img_obj['id'] == lidar_obj['id']: 
                    xmin = float(img_obj['position']['x'])
                    ymin = float(img_obj['position']['y'])
                    xmax = xmin + float(img_obj['dimension']['x'])
                    ymax = ymin + float(img_obj['dimension']['y'])
                    
                    # lidar info 
                    rot_z = lidar_obj['rotation']['z']  #(-pi~pi)
                    # rot_y = -rot_z
                    print(lidar_obj['category'])
                    label = get_label(lidar_obj['category'])
                    x = lidar_obj['position']['x'] 
                    y = lidar_obj['position']['y']
                    z = lidar_obj['position']['z']

                    cl = lidar_obj['dimension']['x']
                    cw = lidar_obj['dimension']['y']
                    ch = lidar_obj['dimension']['z']

                    obj = [label, x, y, z, cl, cw, ch, rot_z, xmin, ymin, xmax, ymax]
                    
                    objs.append(obj)

    #write label txt
    file_lb=open('lines/{}.txt'.format(filename),'w')
    for obj in objs:
        for item in obj:
            file_lb.write(str(item)+ ' ')

    file_lb.close() 

if __name__ == '__main__':
    json_dir =  './selected/json_files/'
    json_files = [x for x in sorted(glob.glob(json_dir+'/*.json'))]
    for json_file in json_files:
        extract_json(json_file)