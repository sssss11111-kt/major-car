from pathlib import Path
import sys, runpy
base=Path(__file__).resolve().parent
sys.path.insert(0,str(base/'tools'))
runpy.run_path('D:\\桌面\\智能车\\major-car-G1-完整下载-20261019\\major-car-c3682dd44872d2dd13b5900b3424d26427f4c492\\major-car-电机标定与循迹接入-20261019\\code\\repair\\tracking_trial.py',run_name='__main__')
