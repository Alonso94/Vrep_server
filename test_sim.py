from env.env_sim import rozum_sim

from stable_baselines.common.policies import CnnLstmPolicy
from stable_baselines import SAC

env=rozum_sim()
model=SAC(CnnLstmPolicy,env,verbose=1)
model.learn(total_timesteps=100)

obs=env.reset()
for i in range(100):
    action,states=model.predict(obs)
    obs,rewards,dones,info=env.step()

RGB_im = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
from matplotlib import pyplot as plt
plt.imshow(img)
