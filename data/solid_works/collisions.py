# Chrono::Engine Python script from SolidWorks
# Assembly: C:\Program Files\SolidWorks Corp\SolidWorks\chronoengine\examples\collisions\portal.SLDASM


import ChronoEngine_PYTHON_core as chrono
import builtins

shapes_dir = 'collisions_shapes/'

if hasattr(builtins, 'exported_system_relpath'):
    shapes_dir = builtins.exported_system_relpath + shapes_dir

exported_items = []

body_0= chrono.ChBodyAuxRefShared()
body_0.SetName('ground')
body_0.SetBodyFixed(1)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRefShared()
body_1.SetName('column-1')
body_1.SetPos(chrono.ChVectorD(0.1,0.1,-2.77555756156289e-17))
body_1.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1.SetMass(5.94162890340754)
body_1.SetInertiaXX(chrono.ChVectorD(0.0828656280407889,0.0140634491293917,0.0828656280407889))
body_1.SetInertiaXY(chrono.ChVectorD(7.3359262007676e-38,-3.61798264958413e-19,-2.89120579226739e-18))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(7.71666145763586e-38,0.200000000001038,4.97386720695722e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_1_1_shape = chrono.ChObjShapeFileShared()
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj')
body_1_1_level = chrono.ChAssetLevelShared()
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1_1_level.GetAssets().push_back(body_1_1_shape)
body_1.GetAssets().push_back(body_1_1_level)

# Collision shape(s)
body_1.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_1.GetCollisionModel().AddCylinder(0.0649999999999999,0.0649999999999999,0.2,chrono.ChVectorD(0,0.2,0),mr)
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(1)

exported_items.append(body_1)



# Rigid body part
body_2= chrono.ChBodyAuxRefShared()
body_2.SetName('capital-2')
body_2.SetPos(chrono.ChVectorD(0.2,0.600000000000001,0.0437815749183074))
body_2.SetRot(chrono.ChQuaternionD(0.707106781186548,0.707106781186547,0,2.4532694666934e-16))
body_2.SetMass(3.48648327019261)
body_2.SetInertiaXX(chrono.ChVectorD(0.0131597753099185,0.0210657426581161,0.0131597753099185))
body_2.SetInertiaXY(chrono.ChVectorD(1.92289812482779e-18,-1.01397495283525e-17,9.13251414033269e-19))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_2.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_2.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_2.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_2.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_2.GetCollisionModel().BuildModel()
body_2.SetCollide(1)

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRefShared()
body_3.SetName('floor^portal-1')
body_3.SetPos(chrono.ChVectorD(0,0,0))
body_3.SetRot(chrono.ChQuaternionD(0,0,0.707106781186548,0.707106781186547))
body_3.SetMass(1031.78399963106)
body_3.SetInertiaXX(chrono.ChVectorD(321.809436313731,935.02961630718,620.098739990989))
body_3.SetInertiaXY(chrono.ChVectorD(-1.14551035283023e-15,1.13487710526994e-13,1.81296560030277e-15))
body_3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.576683292812419,-0.161731094151293,-0.1),chrono.ChQuaternionD(1,0,0,0)))
body_3.SetBodyFixed(1)

# Visualization shape
body_3_1_shape = chrono.ChObjShapeFileShared()
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj')
body_3_1_level = chrono.ChAssetLevelShared()
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_3_1_level.GetAssets().push_back(body_3_1_shape)
body_3.GetAssets().push_back(body_3_1_level)

# Collision shape(s)
body_3.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-8.29127457809838E-17; mr[2,0]=0
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_3.GetCollisionModel().AddBox(1.33902576035516,0.962127013192173,0.1,chrono.ChVectorD(-0.576683292812419,-0.161731094151293,-0.1),mr)
body_3.GetCollisionModel().BuildModel()
body_3.SetCollide(1)

exported_items.append(body_3)



# Rigid body part
body_4= chrono.ChBodyAuxRefShared()
body_4.SetName('capital-1')
body_4.SetPos(chrono.ChVectorD(0,0,0.0437815749183073))
body_4.SetRot(chrono.ChQuaternionD(0,0,0.707106781186548,0.707106781186547))
body_4.SetMass(3.48648327019261)
body_4.SetInertiaXX(chrono.ChVectorD(0.0131597753099185,0.0210657426581161,0.0131597753099185))
body_4.SetInertiaXY(chrono.ChVectorD(8.200353070171e-19,-1.01397495283525e-17,-9.13251414033265e-19))
body_4.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_4.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_4.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_4.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_4.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_4.GetCollisionModel().BuildModel()
body_4.SetCollide(1)

exported_items.append(body_4)



# Rigid body part
body_5= chrono.ChBodyAuxRefShared()
body_5.SetName('capital-6')
body_5.SetPos(chrono.ChVectorD(0.953366585624838,0.600000000000001,-0.04378157491831))
body_5.SetRot(chrono.ChQuaternionD(2.46885013108227e-15,2.22352318441293e-15,-0.707106781186547,0.707106781186548))
body_5.SetMass(3.48648327019261)
body_5.SetInertiaXX(chrono.ChVectorD(0.0131597753099185,0.0210657426581161,0.0131597753099185))
body_5.SetInertiaXY(chrono.ChVectorD(-5.32841767986422e-17,1.01397495283525e-17,-9.13251414033216e-19))
body_5.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_5.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_5.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_5.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_5.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_5.GetCollisionModel().BuildModel()
body_5.SetCollide(1)

exported_items.append(body_5)



# Rigid body part
body_6= chrono.ChBodyAuxRefShared()
body_6.SetName('capital-4')
body_6.SetPos(chrono.ChVectorD(0.620464867730728,0.600000000000001,0.376683292812419))
body_6.SetRot(chrono.ChQuaternionD(0.499999999999999,0.499999999999999,0.500000000000001,-0.500000000000001))
body_6.SetMass(3.48648327019261)
body_6.SetInertiaXX(chrono.ChVectorD(0.0210657426581161,0.0131597753099185,0.0131597753099185))
body_6.SetInertiaXY(chrono.ChVectorD(2.56806393369072e-17,1.82968201781166e-18,-1.01397495283525e-17))
body_6.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_6.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_6.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_6.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_6.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_6.GetCollisionModel().BuildModel()
body_6.SetCollide(1)

exported_items.append(body_6)



# Rigid body part
body_7= chrono.ChBodyAuxRefShared()
body_7.SetName('capital-5')
body_7.SetPos(chrono.ChVectorD(0.873996852035522,0.600000000000001,0.235397062191576))
body_7.SetRot(chrono.ChQuaternionD(0.270598050073097,0.270598050073097,0.653281482438189,-0.653281482438189))
body_7.SetMass(3.48648327019261)
body_7.SetInertiaXX(chrono.ChVectorD(0.0171127589840173,0.0171127589840173,0.0131597753099185))
body_7.SetInertiaXY(chrono.ChVectorD(0.00395298367409879,7.89558609916252e-18,-6.44418520289982e-18))
body_7.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_7.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_7.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_7.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_7.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_7.GetCollisionModel().BuildModel()
body_7.SetCollide(1)

exported_items.append(body_7)



# Rigid body part
body_8= chrono.ChBodyAuxRefShared()
body_8.SetName('capital-8')
body_8.SetPos(chrono.ChVectorD(0.532901717894108,0.600000000000001,-0.376683292812419))
body_8.SetRot(chrono.ChQuaternionD(0.500000000000003,0.500000000000003,-0.499999999999997,0.499999999999998))
body_8.SetMass(3.48648327019261)
body_8.SetInertiaXX(chrono.ChVectorD(0.0210657426581161,0.0131597753099185,0.0131597753099185))
body_8.SetInertiaXY(chrono.ChVectorD(8.08877142603772e-17,3.65618484587806e-18,1.01397495283526e-17))
body_8.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_8.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_8.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_8.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_8.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_8.GetCollisionModel().BuildModel()
body_8.SetCollide(1)

exported_items.append(body_8)



# Rigid body part
body_9= chrono.ChBodyAuxRefShared()
body_9.SetName('capital-7')
body_9.SetPos(chrono.ChVectorD(0.812080355003994,0.600000000000001,-0.297313559223103))
body_9.SetRot(chrono.ChQuaternionD(0.270598050073101,0.270598050073101,-0.653281482438187,0.653281482438187))
body_9.SetMass(3.48648327019261)
body_9.SetInertiaXX(chrono.ChVectorD(0.0171127589840174,0.0171127589840172,0.0131597753099185))
body_9.SetInertiaXY(chrono.ChVectorD(-0.00395298367409879,9.18711863474472e-18,5.15265266731767e-18))
body_9.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_9.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_9.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_9.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_9.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_9.GetCollisionModel().BuildModel()
body_9.SetCollide(1)

exported_items.append(body_9)



# Rigid body part
body_10= chrono.ChBodyAuxRefShared()
body_10.SetName('capital-9')
body_10.SetPos(chrono.ChVectorD(0.279369733589315,0.600000000000001,-0.235397062191574))
body_10.SetRot(chrono.ChQuaternionD(0.65328148243819,0.65328148243819,-0.270598050073095,0.270598050073095))
body_10.SetMass(3.48648327019261)
body_10.SetInertiaXX(chrono.ChVectorD(0.0171127589840172,0.0171127589840174,0.0131597753099185))
body_10.SetInertiaXY(chrono.ChVectorD(0.00395298367409879,-5.15265266731773e-18,9.18711863474467e-18))
body_10.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_10.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_10.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_10.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_10.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_10.GetCollisionModel().BuildModel()
body_10.SetCollide(1)

exported_items.append(body_10)



# Rigid body part
body_11= chrono.ChBodyAuxRefShared()
body_11.SetName('column-2')
body_11.SetPos(chrono.ChVectorD(0.239617303986425,0.1,0.337065988825995))
body_11.SetRot(chrono.ChQuaternionD(0.923879532511286,0,0.382683432365091,0))
body_11.SetMass(5.94162890340754)
body_11.SetInertiaXX(chrono.ChVectorD(0.0828656280407889,0.0140634491293917,0.0828656280407889))
body_11.SetInertiaXY(chrono.ChVectorD(2.0443912215181e-18,0,-2.0443912215181e-18))
body_11.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(7.71666145763586e-38,0.200000000001038,4.97386720695722e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_1_1_shape = chrono.ChObjShapeFileShared()
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj')
body_1_1_level = chrono.ChAssetLevelShared()
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1_1_level.GetAssets().push_back(body_1_1_shape)
body_11.GetAssets().push_back(body_1_1_level)

# Collision shape(s)
body_11.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_11.GetCollisionModel().AddCylinder(0.0649999999999999,0.0649999999999999,0.2,chrono.ChVectorD(0,0.2,0),mr)
body_11.GetCollisionModel().BuildModel()
body_11.SetCollide(1)

exported_items.append(body_11)



# Rigid body part
body_12= chrono.ChBodyAuxRefShared()
body_12.SetName('column-5')
body_12.SetPos(chrono.ChVectorD(1.05336658562484,0.1,-3.30090606717009e-15))
body_12.SetRot(chrono.ChQuaternionD(-3.49148133884313e-15,0,1,0))
body_12.SetMass(5.94162890340754)
body_12.SetInertiaXX(chrono.ChVectorD(0.0828656280407889,0.0140634491293917,0.0828656280407889))
body_12.SetInertiaXY(chrono.ChVectorD(-2.01892555001756e-32,-3.61798264958461e-19,2.89120579226739e-18))
body_12.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(7.71666145763586e-38,0.200000000001038,4.97386720695722e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_1_1_shape = chrono.ChObjShapeFileShared()
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj')
body_1_1_level = chrono.ChAssetLevelShared()
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1_1_level.GetAssets().push_back(body_1_1_shape)
body_12.GetAssets().push_back(body_1_1_level)

# Collision shape(s)
body_12.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_12.GetCollisionModel().AddCylinder(0.0649999999999999,0.0649999999999999,0.2,chrono.ChVectorD(0,0.2,0),mr)
body_12.GetCollisionModel().BuildModel()
body_12.SetCollide(1)

exported_items.append(body_12)



# Rigid body part
body_13= chrono.ChBodyAuxRefShared()
body_13.SetName('column-4')
body_13.SetPos(chrono.ChVectorD(0.913749281638415,0.1,0.337065988825992))
body_13.SetRot(chrono.ChQuaternionD(0.382683432365087,0,0.923879532511288,0))
body_13.SetMass(5.94162890340754)
body_13.SetInertiaXX(chrono.ChVectorD(0.0828656280407889,0.0140634491293917,0.0828656280407889))
body_13.SetInertiaXY(chrono.ChVectorD(2.04439122151809e-18,0,2.04439122151811e-18))
body_13.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(7.71666145763586e-38,0.200000000001038,4.97386720695722e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_1_1_shape = chrono.ChObjShapeFileShared()
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj')
body_1_1_level = chrono.ChAssetLevelShared()
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1_1_level.GetAssets().push_back(body_1_1_shape)
body_13.GetAssets().push_back(body_1_1_level)

# Collision shape(s)
body_13.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_13.GetCollisionModel().AddCylinder(0.0649999999999999,0.0649999999999999,0.2,chrono.ChVectorD(0,0.2,0),mr)
body_13.GetCollisionModel().BuildModel()
body_13.SetCollide(1)

exported_items.append(body_13)



# Rigid body part
body_14= chrono.ChBodyAuxRefShared()
body_14.SetName('column-3')
body_14.SetPos(chrono.ChVectorD(0.576683292812421,0.1,0.476683292812419))
body_14.SetRot(chrono.ChQuaternionD(0.707106781186546,0,0.707106781186549,0))
body_14.SetMass(5.94162890340754)
body_14.SetInertiaXX(chrono.ChVectorD(0.0828656280407889,0.0140634491293917,0.0828656280407889))
body_14.SetInertiaXY(chrono.ChVectorD(2.89120579226739e-18,3.61798264958412e-19,1.00946644297188e-32))
body_14.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(7.71666145763586e-38,0.200000000001038,4.97386720695722e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_1_1_shape = chrono.ChObjShapeFileShared()
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj')
body_1_1_level = chrono.ChAssetLevelShared()
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1_1_level.GetAssets().push_back(body_1_1_shape)
body_14.GetAssets().push_back(body_1_1_level)

# Collision shape(s)
body_14.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_14.GetCollisionModel().AddCylinder(0.0649999999999999,0.0649999999999999,0.2,chrono.ChVectorD(0,0.2,0),mr)
body_14.GetCollisionModel().BuildModel()
body_14.SetCollide(1)

exported_items.append(body_14)



# Rigid body part
body_15= chrono.ChBodyAuxRefShared()
body_15.SetName('capital-3')
body_15.SetPos(chrono.ChVectorD(0.341286230620842,0.600000000000001,0.297313559223102))
body_15.SetRot(chrono.ChQuaternionD(0.653281482438188,0.653281482438188,0.270598050073099,-0.270598050073099))
body_15.SetMass(3.48648327019261)
body_15.SetInertiaXX(chrono.ChVectorD(0.0171127589840173,0.0171127589840173,0.0131597753099185))
body_15.SetInertiaXY(chrono.ChVectorD(-0.0039529836740988,-6.44418520289984e-18,-7.89558609916251e-18))
body_15.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_15.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_15.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_15.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_15.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_15.GetCollisionModel().BuildModel()
body_15.SetCollide(1)

exported_items.append(body_15)



# Rigid body part
body_16= chrono.ChBodyAuxRefShared()
body_16.SetName('column-6')
body_16.SetPos(chrono.ChVectorD(0.91374928163841,0.1,-0.337065988825997))
body_16.SetRot(chrono.ChQuaternionD(-0.382683432365094,0,0.923879532511285,0))
body_16.SetMass(5.94162890340754)
body_16.SetInertiaXX(chrono.ChVectorD(0.0828656280407889,0.0140634491293917,0.0828656280407889))
body_16.SetInertiaXY(chrono.ChVectorD(-2.04439122151812e-18,0,2.04439122151808e-18))
body_16.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(7.71666145763586e-38,0.200000000001038,4.97386720695722e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_1_1_shape = chrono.ChObjShapeFileShared()
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj')
body_1_1_level = chrono.ChAssetLevelShared()
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1_1_level.GetAssets().push_back(body_1_1_shape)
body_16.GetAssets().push_back(body_1_1_level)

# Collision shape(s)
body_16.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_16.GetCollisionModel().AddCylinder(0.0649999999999999,0.0649999999999999,0.2,chrono.ChVectorD(0,0.2,0),mr)
body_16.GetCollisionModel().BuildModel()
body_16.SetCollide(1)

exported_items.append(body_16)



# Rigid body part
body_17= chrono.ChBodyAuxRefShared()
body_17.SetName('column-7')
body_17.SetPos(chrono.ChVectorD(0.576683292812414,0.1,-0.476683292812419))
body_17.SetRot(chrono.ChQuaternionD(0.707106781186551,0,-0.707106781186544,0))
body_17.SetMass(5.94162890340754)
body_17.SetInertiaXX(chrono.ChVectorD(0.0828656280407889,0.0140634491293917,0.0828656280407889))
body_17.SetInertiaXY(chrono.ChVectorD(-2.89120579226739e-18,3.61798264958461e-19,-3.02838465706323e-32))
body_17.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(7.71666145763586e-38,0.200000000001038,4.97386720695722e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_1_1_shape = chrono.ChObjShapeFileShared()
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj')
body_1_1_level = chrono.ChAssetLevelShared()
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1_1_level.GetAssets().push_back(body_1_1_shape)
body_17.GetAssets().push_back(body_1_1_level)

# Collision shape(s)
body_17.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_17.GetCollisionModel().AddCylinder(0.0649999999999999,0.0649999999999999,0.2,chrono.ChVectorD(0,0.2,0),mr)
body_17.GetCollisionModel().BuildModel()
body_17.SetCollide(1)

exported_items.append(body_17)



# Rigid body part
body_18= chrono.ChBodyAuxRefShared()
body_18.SetName('column-8')
body_18.SetPos(chrono.ChVectorD(0.239617303986421,0.1,-0.33706598882599))
body_18.SetRot(chrono.ChQuaternionD(0.923879532511289,0,-0.382683432365084,0))
body_18.SetMass(5.94162890340754)
body_18.SetInertiaXX(chrono.ChVectorD(0.0828656280407889,0.0140634491293917,0.0828656280407889))
body_18.SetInertiaXY(chrono.ChVectorD(-2.04439122151807e-18,0,-2.04439122151812e-18))
body_18.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(7.71666145763586e-38,0.200000000001038,4.97386720695722e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_1_1_shape = chrono.ChObjShapeFileShared()
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj')
body_1_1_level = chrono.ChAssetLevelShared()
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1_1_level.GetAssets().push_back(body_1_1_shape)
body_18.GetAssets().push_back(body_1_1_level)

# Collision shape(s)
body_18.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_18.GetCollisionModel().AddCylinder(0.0649999999999999,0.0649999999999999,0.2,chrono.ChVectorD(0,0.2,0),mr)
body_18.GetCollisionModel().BuildModel()
body_18.SetCollide(1)

exported_items.append(body_18)



# Rigid body part
body_19= chrono.ChBodyAuxRefShared()
body_19.SetName('capital-10')
body_19.SetPos(chrono.ChVectorD(0.199864874383533,0,0.438734915460412))
body_19.SetRot(chrono.ChQuaternionD(-0.270598050073099,0.270598050073099,0.653281482438188,0.653281482438188))
body_19.SetMass(3.48648327019261)
body_19.SetInertiaXX(chrono.ChVectorD(0.0171127589840173,0.0171127589840173,0.0131597753099185))
body_19.SetInertiaXY(chrono.ChVectorD(0.0039529836740988,-7.81565191882229e-18,6.52411938324006e-18))
body_19.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_19.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_19.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_19.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_19.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_19.GetCollisionModel().BuildModel()
body_19.SetCollide(1)

exported_items.append(body_19)



# Rigid body part
body_20= chrono.ChBodyAuxRefShared()
body_20.SetName('capital-11')
body_20.SetPos(chrono.ChVectorD(0.620464867730728,0,0.576683292812419))
body_20.SetRot(chrono.ChQuaternionD(0.500000000000001,-0.500000000000001,-0.499999999999999,-0.499999999999999))
body_20.SetMass(3.48648327019261)
body_20.SetInertiaXX(chrono.ChVectorD(0.0210657426581161,0.0131597753099185,0.0131597753099185))
body_20.SetInertiaXY(chrono.ChVectorD(-2.84235727687521e-17,-9.1325141403323e-19,1.01397495283525e-17))
body_20.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_20.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_20.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_20.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_20.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_20.GetCollisionModel().BuildModel()
body_20.SetCollide(1)

exported_items.append(body_20)



# Rigid body part
body_21= chrono.ChBodyAuxRefShared()
body_21.SetName('capital-12')
body_21.SetPos(chrono.ChVectorD(1.01541820827283,0,0.376818418428885))
body_21.SetRot(chrono.ChQuaternionD(0.653281482438189,-0.653281482438189,-0.270598050073097,-0.270598050073097))
body_21.SetMass(3.48648327019261)
body_21.SetInertiaXX(chrono.ChVectorD(0.0171127589840173,0.0171127589840173,0.0131597753099185))
body_21.SetInertiaXY(chrono.ChVectorD(-0.00395298367409879,6.52411938324009e-18,7.81565191882227e-18))
body_21.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_21.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_21.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_21.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_21.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_21.GetCollisionModel().BuildModel()
body_21.SetCollide(1)

exported_items.append(body_21)



# Rigid body part
body_22= chrono.ChBodyAuxRefShared()
body_22.SetName('capital-13')
body_22.SetPos(chrono.ChVectorD(1.15336658562484,0,-0.0437815749183113))
body_22.SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,2.46885013108227e-15,2.46885013108227e-15))
body_22.SetMass(3.48648327019261)
body_22.SetInertiaXX(chrono.ChVectorD(0.0131597753099185,0.0210657426581161,0.0131597753099185))
body_22.SetInertiaXY(chrono.ChVectorD(5.60271102304871e-17,1.01397495283525e-17,9.13251414033194e-19))
body_22.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_22.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_22.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_22.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_22.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_22.GetCollisionModel().BuildModel()
body_22.SetCollide(1)

exported_items.append(body_22)



# Rigid body part
body_23= chrono.ChBodyAuxRefShared()
body_23.SetName('capital-14')
body_23.SetPos(chrono.ChVectorD(0.953501711241302,0,-0.438734915460414))
body_23.SetRot(chrono.ChQuaternionD(0.653281482438187,-0.653281482438187,0.270598050073101,0.270598050073101))
body_23.SetMass(3.48648327019261)
body_23.SetInertiaXX(chrono.ChVectorD(0.0171127589840174,0.0171127589840172,0.0131597753099185))
body_23.SetInertiaXY(chrono.ChVectorD(0.00395298367409879,7.81565191882224e-18,-6.52411938324012e-18))
body_23.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_23.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_23.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_23.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_23.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_23.GetCollisionModel().BuildModel()
body_23.SetCollide(1)

exported_items.append(body_23)



# Rigid body part
body_24= chrono.ChBodyAuxRefShared()
body_24.SetName('capital-15')
body_24.SetPos(chrono.ChVectorD(0.532901717894106,0,-0.576683292812419))
body_24.SetRot(chrono.ChQuaternionD(0.499999999999997,-0.499999999999997,0.500000000000003,0.500000000000003))
body_24.SetMass(3.48648327019261)
body_24.SetInertiaXX(chrono.ChVectorD(0.0210657426581161,0.0131597753099185,0.0131597753099185))
body_24.SetInertiaXY(chrono.ChVectorD(-8.36306476922221e-17,9.13251414033159e-19,-1.01397495283525e-17))
body_24.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_24.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_24.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_24.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_24.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_24.GetCollisionModel().BuildModel()
body_24.SetCollide(1)

exported_items.append(body_24)



# Rigid body part
body_25= chrono.ChBodyAuxRefShared()
body_25.SetName('capital-16')
body_25.SetPos(chrono.ChVectorD(0.137948377352004,0,-0.376818418428882))
body_25.SetRot(chrono.ChQuaternionD(0.270598050073095,-0.270598050073095,0.65328148243819,0.65328148243819))
body_25.SetMass(3.48648327019261)
body_25.SetInertiaXX(chrono.ChVectorD(0.0171127589840172,0.0171127589840174,0.0131597753099185))
body_25.SetInertiaXY(chrono.ChVectorD(-0.00395298367409879,-6.52411938324014e-18,-7.81565191882222e-18))
body_25.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.1,-0.0437815749183073,0.0461040281578376),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFileShared()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevelShared()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_25.GetAssets().push_back(body_2_1_level)

# Collision shape(s)
body_25.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_25.GetCollisionModel().AddCylinder(0.0857670867875105,0.0857670867875105,0.01,chrono.ChVectorD(-0.1,-0.0437815749183074,0.09),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-16; mr[2,0]=0
mr[0,1]=-1.22464679914735E-16; mr[1,1]=1; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1
body_25.GetCollisionModel().AddBox(0.1,0.1,0.04,chrono.ChVectorD(-0.1,-0.0437815749183073,0.04),mr)
body_25.GetCollisionModel().BuildModel()
body_25.SetCollide(1)

exported_items.append(body_25)



