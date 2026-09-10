import numpy as np
from hand_retargeting import HandRetargeting, HandType   # 在 teleop/robot_control 下运行

hr = HandRetargeting(HandType.O10)
print("left retarget joint_names =", hr.left_retargeting_joint_names)
print("left_indices shape =", hr.left_indices.shape)

def synth(open_hand: bool):
    # 合成 25x3 人手 landmark：腕在原点，5 指沿 +x 展开；open=伸直, fist=指尖收拢
    lm = np.zeros((25,3)); lm[0]=[0,0,0]
    tips  = [4,9,14,19,24]; 
    for k,tip in enumerate(tips):
        spread = (k-2)*0.03                       # y 方向指间展开
        reach  = 0.18 if open_hand else 0.06      # 伸直 vs 握拳
        for j in range(1,5):
            idx = tip-4+j-1 if tip!=4 else j      # 粗略填充各指关节
            if 0<=idx<25: lm[idx]=[reach*j/4, spread, 0]
        lm[tip]=[reach, spread, 0]
    return lm

for open_hand in (True, False):
    lm = synth(open_hand)
    ref = lm[hr.left_indices[1,:]] - lm[hr.left_indices[0,:]]
    q = hr.left_retargeting.retarget(ref)
    print(f"open={open_hand}  q_dim={q.shape}  q=", np.round(q,3))