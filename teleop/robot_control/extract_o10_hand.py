#!/usr/bin/env python3
# 从完整 G2 URDF 抽取独立 O10 左/右手 URDF：以 hand_X_base_link 为根，保留 joint(含 mimic/limit)，去掉 visual/collision。
import os, copy, xml.etree.ElementTree as ET

SRC = "/opt/workspace/G2_Robot/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB_o10_t2.urdf"
OUT_DIR = "/opt/workspace/xr_teleoperate/assets/o10_hand"

def extract(src, root_link, out_path):
    robot = ET.parse(src).getroot()
    joints = robot.findall("joint"); links = robot.findall("link")
    jtrip = [(j, j.find("parent").get("link"), j.find("child").get("link")) for j in joints]
    keep = {root_link}; changed = True
    while changed:                       # 从 root 沿 parent->child 扩散
        changed = False
        for j, p, c in jtrip:
            if p in keep and c not in keep:
                keep.add(c); changed = True
    keep_j = [j for j, p, c in jtrip if p in keep and c in keep]
    new = ET.Element("robot", {"name": root_link})
    for l in links:                      # 只留 link 名 + inertial，去掉 visual/collision/mesh
        if l.get("name") in keep:
            nl = ET.SubElement(new, "link", {"name": l.get("name")})
            ine = l.find("inertial")
            if ine is not None: nl.append(copy.deepcopy(ine))
    for j in keep_j: new.append(copy.deepcopy(j))
    ET.ElementTree(new).write(out_path, encoding="utf-8", xml_declaration=True)
    print(f"[ok] {out_path}: links={len(keep)} joints={len(keep_j)}")

if __name__ == "__main__":
    os.makedirs(OUT_DIR, exist_ok=True)
    extract(SRC, "hand_l_base_link", f"{OUT_DIR}/o10_left.urdf")
    extract(SRC, "hand_r_base_link", f"{OUT_DIR}/o10_right.urdf")