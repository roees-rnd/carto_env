import rosbag,glob,sys,os

bagDir = sys.argv[1]
bagFiles = glob.glob(bagDir+os.sep+'*.bag')

with rosbag.Bag(os.path.join(bagDir,'merged.bag'), 'w') as outbag:
    for bagFile in bagFiles:
        for topic, msg, t in rosbag.Bag(bagFile).read_messages():
            outbag.write(topic, msg, t)

