import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm


planar3_model = rtb.models.DH.Planar3()

planar3_model.teach(q=[0, 0, 0])
