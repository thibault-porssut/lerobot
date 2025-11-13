
FOR Mac
STEP 1
docker run --rm --platform linux/amd64 -p 8888:8888 -p 6080:6080 -p 50051:50051 -p 50065:50065 --name reachy2_temp --entrypoint /package/launch.sh pollenrobotics/reachy2_core:1.7.8.5_development start_rviz:=true start_sdk_server:=true fake:=true mujoco:=true

STEP 2
mjpython reachy2_mujoco_server.py --model reachy2_mujoco_assets/scenes/fruits_scene.xml

FOR ALL PLATFORMS

STEP 3
 python -m lerobot.scripts.task_server

STEP 4
python -m lerobot.scripts.lerobot_record_extended \
    --robot.type=reachy2 \
    --robot.ip_address=localhost \
    --robot.id=r2-0000 \
    --robot.use_external_commands=false \
    --robot.with_mobile_base=false \
    --teleop.type=reachy2_teleoperator \
    --teleop.ip_address=localhost \
    --teleop.with_mobile_base=true \
    --dataset.repo_id=fenduil/record_test2\
    --dataset.single_task="Reachy 2 recording test" \
    --dataset.num_episodes=3 \
    --dataset.episode_time_s=9999 \
    --dataset.fps=15 \
    --dataset.push_to_hub=true \
    --dataset.private=false \
    --display_data=true  \
   --dataset.reset_time_s=5



