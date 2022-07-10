source ~/anaconda3/bin/activate 
conda activate sfm
source /home/crescentrose/py3Ros_ws/devel/setup.bash
cd /home/crescentrose/py3Ros_ws/src/sc_depth_pl-master/
python rgbd.py --config configs/v2/nyu.txt \
--input_dir demo/input/ \
--output_dir demo/output/ \
--ckpt_path ckpts/rgbd.ckpt \
--save-vis --save-depth
