# @Description: Custom communication type for this competition
# @Author: lifuguan
# @Date: 2019-10-12 17:21:04
# @LastEditTime: 2019-10-12 15:37:19
# @LastEditors: Please set LastEditors

# 0 : do nothing
# 1 : detect the material area
# 2 : tracking the specific cylinder
int32 cv_mission_type


# 0 : red
# 1 : blue
# 2 : green
int32 cv_target_color

# the queue of the material cylinders
# 0 : red
# 1 : blue
# 2 : green
int32[] material_queue