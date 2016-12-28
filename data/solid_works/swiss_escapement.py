# Chrono::Engine Python script from SolidWorks
# Assembly: C:\tasora\lavori\orologio_audemar\CAD_swiss_escapement\escapement.SLDASM


import ChronoEngine_python_core as chrono
import builtins

shapes_dir = 'swiss_escapement_shapes/'

if hasattr(builtins, 'exported_system_relpath'):
    shapes_dir = builtins.exported_system_relpath + shapes_dir

exported_items = []

body_0= chrono.ChBodyAuxRef()
body_0.SetName('ground')
body_0.SetBodyFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRef()
body_1.SetName('escape_wheel^escapement-1')
body_1.SetPos(chrono.ChVectorD(0,0.000381819559584939,0))
body_1.SetRot(chrono.ChQuaternionD(0,0,0.707106781186548,0.707106781186547))
body_1.SetMass(0.385093622816182)
body_1.SetInertiaXX(chrono.ChVectorD(0.000614655341550614,0.00114774663635329,0.000614655341550614))
body_1.SetInertiaXY(chrono.ChVectorD(1.04945260437012e-19,-5.29910899706164e-19,5.85921324575995e-19))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-1.29340068058665e-17,4.10138104133823e-17,0.00633921901294084),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_1_1_shape = chrono.ChObjShapeFile()
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj')
body_1_1_level = chrono.ChAssetLevel()
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1_1_level.GetAssets().push_back(body_1_1_shape)
body_1.GetAssets().push_back(body_1_1_level)

# Collision shape(s)
body_1.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=-0.947036648244772; mr[1,1]=0.321125500204061; mr[2,1]=0
mr[0,2]=0.321125500204061; mr[1,2]=0.947036648244772; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.0025,0.00328030945702576,0.002,chrono.ChVectorD(0.0120114645617556,-0.0887792305476897,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=-0.734547520266048; mr[1,1]=0.678557249221464; mr[2,1]=0
mr[0,2]=0.678557249221464; mr[1,2]=0.734547520266048; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.0025,0.00328030945702575,0.002,chrono.ChVectorD(-0.025136747317793,-0.0859893655741196,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=-0.395048452878596; mr[1,1]=0.918660285349393; mr[2,1]=0
mr[0,2]=0.918660285349393; mr[1,2]=0.395048452878596; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.0025,0.00328030945702575,0.002,chrono.ChVectorD(-0.0579385872259149,-0.0683311581039222,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0.0127580809140941; mr[1,1]=0.999918612373722; mr[2,1]=0
mr[0,2]=0.999918612373722; mr[1,2]=-0.0127580809140941; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.0025,0.00328030945702575,0.002,chrono.ChVectorD(-0.0807223190471352,-0.0388578726284734,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0.418358626613207; mr[1,1]=0.908281927343218; mr[2,1]=0
mr[0,2]=0.908281927343218; mr[1,2]=-0.418358626613207; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.0025,0.00328030945702576,0.002,chrono.ChVectorD(-0.0895484285658595,-0.00266570796287103,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0.75162116510209; mr[1,1]=0.659595045592807; mr[2,1]=0
mr[0,2]=0.659595045592807; mr[1,2]=-0.75162116510209; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.0025,0.00328030945702575,0.002,chrono.ChVectorD(-0.0828908012636124,0.0339873818267083,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0.954921575880902; mr[1,1]=0.296858188226525; mr[2,1]=0
mr[0,2]=0.296858188226525; mr[1,2]=-0.954921575880902; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.0025,0.00328030945702576,0.002,chrono.ChVectorD(-0.0619006013835978,0.0647637445327791,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0.993107370999733; mr[1,1]=-0.117208146756099; mr[2,1]=0
mr[0,2]=-0.117208146756099; mr[1,2]=-0.993107370999733; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.0025,0.00328030945702576,0.002,chrono.ChVectorD(-0.0302072251750497,0.084341867448984,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0.85957587957548; mr[1,1]=-0.511008128362006; mr[2,1]=0
mr[0,2]=-0.511008128362006; mr[1,2]=-0.85957587957548; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.0025,0.00328030945702575,0.00199999999999999,chrono.ChVectorD(0.00670925469029009,0.0893365152614482,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0.577415909570911; mr[1,1]=-0.816450162211018; mr[2,1]=0
mr[0,2]=-0.816450162211018; mr[1,2]=-0.577415909570911; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.0025,0.00328030945702576,0.00199999999999999,chrono.ChVectorD(0.0424656434680133,0.0788840679884458,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=0.195415483142673; mr[1,1]=-0.980720545796873; mr[2,1]=0
mr[0,2]=-0.980720545796873; mr[1,2]=-0.195415483142673; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.0025,0.00328030945702576,0.002,chrono.ChVectorD(0.0708793367018574,0.0547918487209811,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=-0.220374055614862; mr[1,1]=-0.975415437447992; mr[2,1]=0
mr[0,2]=-0.975415437447992; mr[1,2]=0.220374055614862; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.0025,0.00328030945702575,0.002,chrono.ChVectorD(0.0870373487013913,0.0212256210413399,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=-0.598058918121146; mr[1,1]=-0.801452138593294; mr[2,1]=0
mr[0,2]=-0.801452138593294; mr[1,2]=0.598058918121146; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.0025,0.00328030945702576,0.002,chrono.ChVectorD(0.0881458124009648,-0.0160107093450625,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=-0.872333960489579; mr[1,1]=-0.48891048401171; mr[2,1]=0
mr[0,2]=-0.48891048401171; mr[1,2]=0.872333960489579; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.0025,0.00328030945702575,0.002,chrono.ChVectorD(0.0740130643568451,-0.0504786426329755,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1
mr[0,1]=-0.995774536184123; mr[1,1]=-0.091831765132195; mr[2,1]=0
mr[0,2]=-0.091831765132195; mr[1,2]=0.995774536184123; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.0025,0.00328030945702576,0.002,chrono.ChVectorD(0.0470827850978458,-0.0762183600255753,0.0025),mr)
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(True)

exported_items.append(body_1)



# Rigid body part
body_2= chrono.ChBodyAuxRef()
body_2.SetName('anchor^escapement-1')
body_2.SetPos(chrono.ChVectorD(-0.0488143557024324,-0.0285802308259733,-0.0120367885808735))
body_2.SetRot(chrono.ChQuaternionD(0.970918101148949,0,-0.239411864495723,0))
body_2.SetMass(0.275550842295851)
body_2.SetInertiaXX(chrono.ChVectorD(0.000366247345114445,0.000668606289866648,0.000310404591044085))
body_2.SetInertiaXY(chrono.ChVectorD(1.2259017591113e-06,0.000183954121861423,9.45801322659851e-07))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.00829010549593409,0.0327771384599818,-0.111831115861805),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFile()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevel()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0488143557024307,0.0278474043945381,-0.0120367885808807))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.169289752881441,0.169289752881441,0.686542773299188,0.686542773299188))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_2.GetAssets().push_back(body_2_1_level)

# Visualization shape
body_2_2_shape = chrono.ChObjShapeFile()
body_2_2_shape.SetFilename(shapes_dir +'body_2_2.obj')
body_2_2_level = chrono.ChAssetLevel()
body_2_2_level.GetFrame().SetPos(chrono.ChVectorD(0.0488143557024307,0.0378474043945381,-0.0120367885808807))
body_2_2_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.169289752881441,0.169289752881441,0.686542773299188,0.686542773299188))
body_2_2_level.GetAssets().push_back(body_2_2_shape)
body_2.GetAssets().push_back(body_2_2_level)

# Collision shape(s)
body_2.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_2.GetCollisionModel().AddCylinder(0.00152838632048226,0.00152838632048226,0.0015,chrono.ChVectorD(-0.0520321728218582,0.0257302492262216,-0.187268430320609),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.841992572625772; mr[1,0]=0; mr[2,0]=0.539489117260983
mr[0,1]=-0.539489117260985; mr[1,1]=0; mr[2,1]=-0.841992572625771
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0
body_2.GetCollisionModel().AddBox(0.00239999309720335,0.00250923730353967,0.00280857758415823,chrono.ChVectorD(-0.0401200925870508,0.0350388268103799,-0.178717071200898),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0.529523859525646; mr[1,0]=0; mr[2,0]=0.848295044305379
mr[0,1]=-0.848295044305379; mr[1,1]=0; mr[2,1]=0.529523859525645
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0
body_2.GetCollisionModel().AddBox(0.00250923730353971,0.00239999309720335,0.00280857758415823,chrono.ChVectorD(-0.0494110785736743,0.0350388268103799,-0.172841038928428),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0.845149386603263; mr[1,0]=0; mr[2,0]=-0.53453018092913
mr[0,1]=0.534530180929129; mr[1,1]=0; mr[2,1]=0.845149386603264
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0
body_2.GetCollisionModel().AddBox(0.00350000000000001,0.0243835708612008,0.00280857758415823,chrono.ChVectorD(-0.0224331359558976,0.0350388268103799,-0.140469187277248),mr)
pt_vect = chrono.vector_ChVectorD()
pt_vect.push_back(chrono.ChVectorD(0.0521295297003961,0.0258474043945381,-0.109787208704166))
pt_vect.push_back(chrono.ChVectorD(0.0585680663475354,0.0258474043945381,-0.13676931754083))
pt_vect.push_back(chrono.ChVectorD(0.0702403529187583,0.0258474043945381,-0.133984048038744))
pt_vect.push_back(chrono.ChVectorD(0.0633542807043835,0.0258474043945381,-0.10512644263427))
pt_vect.push_back(chrono.ChVectorD(0.0521295297003961,0.0398474043945381,-0.109787208704166))
pt_vect.push_back(chrono.ChVectorD(0.0585680663475354,0.0398474043945381,-0.13676931754083))
pt_vect.push_back(chrono.ChVectorD(0.0702403529187583,0.0398474043945381,-0.133984048038744))
pt_vect.push_back(chrono.ChVectorD(0.0633542807043835,0.0398474043945381,-0.10512644263427))
body_2.GetCollisionModel().AddConvexHull(pt_vect)
pt_vect = chrono.vector_ChVectorD()
pt_vect.push_back(chrono.ChVectorD(-0.0448398581045523,0.0258474043945381,-0.0794862238349664))
pt_vect.push_back(chrono.ChVectorD(-0.0362924460942842,0.0258474043945381,-0.0879089162742476))
pt_vect.push_back(chrono.ChVectorD(-0.0169874992605579,0.0258474043945381,-0.0683181101735014))
pt_vect.push_back(chrono.ChVectorD(-0.026134463034664,0.0258474043945381,-0.0605038474001435))
pt_vect.push_back(chrono.ChVectorD(-0.0448398581045523,0.0398474043945381,-0.0794862238349664))
pt_vect.push_back(chrono.ChVectorD(-0.0362924460942842,0.0398474043945381,-0.0879089162742476))
pt_vect.push_back(chrono.ChVectorD(-0.0169874992605579,0.0398474043945381,-0.0683181101735014))
pt_vect.push_back(chrono.ChVectorD(-0.026134463034664,0.0398474043945381,-0.0605038474001435))
body_2.GetCollisionModel().AddConvexHull(pt_vect)
body_2.GetCollisionModel().BuildModel()
body_2.SetCollide(True)

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRef()
body_3.SetName('balance^escapement-1')
body_3.SetPos(chrono.ChVectorD(-0.0112309579216444,3.57656060958264e-05,-0.000288097650912942))
body_3.SetRot(chrono.ChQuaternionD(0.725007055701766,0,0.688741438554888,0))
body_3.SetMass(1.81117045126479)
body_3.SetInertiaXX(chrono.ChVectorD(0.0202876017800519,0.0412119638526683,0.0212193790010741))
body_3.SetInertiaXY(chrono.ChVectorD(-5.11388221585704e-08,0.000110316054480198,5.50088243531004e-07))
body_3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.21901864850551,0.0330968348592532,9.59245261293635e-07),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_3_1_shape = chrono.ChObjShapeFile()
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj')
body_3_1_level = chrono.ChAssetLevel()
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_3_1_level.GetAssets().push_back(body_3_1_shape)
body_3.GetAssets().push_back(body_3_1_level)

# Collision shape(s)
body_3.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0.973704418632404; mr[1,0]=0; mr[2,0]=0.227815067841729
mr[0,1]=-0.227815067841728; mr[1,1]=0; mr[2,1]=0.973704418632405
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0
body_3.GetCollisionModel().AddBox(0.000580726246669864,0.00255446596440967,0.0075,chrono.ChVectorD(0.190889475660623,0.0103234187261071,-0.00657694561809265),mr)
body_3.GetCollisionModel().BuildModel()
body_3.SetCollide(True)

exported_items.append(body_3)



# Rigid body part
body_4= chrono.ChBodyAuxRef()
body_4.SetName('truss^escapement-1')
body_4.SetPos(chrono.ChVectorD(0,0,0))
body_4.SetRot(chrono.ChQuaternionD(0,0,0.707106781186548,0.707106781186547))
body_4.SetMass(1.02644190893078)
body_4.SetInertiaXX(chrono.ChVectorD(0.00421426808616481,0.00449296635132803,0.000334399259807225))
body_4.SetInertiaXY(chrono.ChVectorD(-1.29040175558272e-20,-1.6130021944784e-19,0.000147522492869351))
body_4.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,-0.120998995444052,-0.0214658323718929),chrono.ChQuaternionD(1,0,0,0)))
body_4.SetBodyFixed(True)

# Visualization shape
body_4_1_shape = chrono.ChObjShapeFile()
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj')
body_4_1_level = chrono.ChAssetLevel()
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_4_1_level.GetAssets().push_back(body_4_1_shape)
body_4.GetAssets().push_back(body_4_1_level)

# Collision shape(s)
body_4.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_4.GetCollisionModel().AddCylinder(0.00307962367981624,0.00307962367981624,0.012,chrono.ChVectorD(-0.0119269719437114,-0.166883602750055,-0.000732826431435216),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_4.GetCollisionModel().AddCylinder(0.00307962367981624,0.00307962367981624,0.012,chrono.ChVectorD(0.0119269719437114,-0.166883602750055,-0.000732826431435216),mr)
body_4.GetCollisionModel().BuildModel()
body_4.SetCollide(True)

exported_items.append(body_4)




# Mate constraint: Concentrico1 [MateConcentric]
#   Entity 0: C::E name: body_1 , SW name: escape_wheel^escapement-1 ,  SW ref.type:1
#   Entity 1: C::E name: body_4 , SW name: truss^escapement-1 ,  SW ref.type:2

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0,-0.0296181804404151,0)
dA = chrono.ChVectorD(0,-1,0)
cB = chrono.ChVectorD(0,-0.03,0)
dB = chrono.ChVectorD(0,1,0)
link_1.SetFlipped(True)
link_1.Initialize(body_1,body_4,False,cA,cB,dA,dB)
link_1.SetName("Concentrico1")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0,-0.0296181804404151,0)
cB = chrono.ChVectorD(0,-0.03,0)
dA = chrono.ChVectorD(0,-1,0)
dB = chrono.ChVectorD(0,1,0)
link_2.Initialize(body_1,body_4,False,cA,cB,dA,dB)
link_2.SetName("Concentrico1")
exported_items.append(link_2)


# Mate constraint: Distanza2 [MateDistanceDim]
#   Entity 0: C::E name: body_1 , SW name: escape_wheel^escapement-1 ,  SW ref.type:2
#   Entity 1: C::E name: body_4 , SW name: truss^escapement-1 ,  SW ref.type:2

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0,0.000381819559584939,0)
cB = chrono.ChVectorD(0.03,-0.0256181804404151,0.0274036523869519)
dA = chrono.ChVectorD(0,-1,0)
dB = chrono.ChVectorD(0,1,-1.05833389757388e-16)
link_3.Initialize(body_1,body_4,False,cA,cB,dB)
link_3.SetDistance(0.026)
link_3.SetName("Distanza2")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0,0.000381819559584939,0)
dA = chrono.ChVectorD(0,-1,0)
cB = chrono.ChVectorD(0.03,-0.0256181804404151,0.0274036523869519)
dB = chrono.ChVectorD(0,1,-1.05833389757388e-16)
link_4.SetFlipped(True)
link_4.Initialize(body_1,body_4,False,cA,cB,dA,dB)
link_4.SetName("Distanza2")
exported_items.append(link_4)


# Mate constraint: Concentrico2 [MateConcentric]
#   Entity 0: C::E name: body_3 , SW name: balance^escapement-1 ,  SW ref.type:2
#   Entity 1: C::E name: body_4 , SW name: truss^escapement-1 ,  SW ref.type:2

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.2490009027033e-16,3.5777905607493e-05,-0.219053185080869)
dA = chrono.ChVectorD(0,-1,0)
cB = chrono.ChVectorD(0,-0.03,-0.21905318508087)
dB = chrono.ChVectorD(0,1,0)
link_5.SetFlipped(True)
link_5.Initialize(body_3,body_4,False,cA,cB,dA,dB)
link_5.SetName("Concentrico2")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateGeneric()
link_6.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.2490009027033e-16,3.5777905607493e-05,-0.219053185080869)
cB = chrono.ChVectorD(0,-0.03,-0.21905318508087)
dA = chrono.ChVectorD(0,-1,0)
dB = chrono.ChVectorD(0,1,0)
link_6.Initialize(body_3,body_4,False,cA,cB,dA,dB)
link_6.SetName("Concentrico2")
exported_items.append(link_6)


# Mate constraint: Distanza3 [MateDistanceDim]
#   Entity 0: C::E name: body_4 , SW name: truss^escapement-1 ,  SW ref.type:2
#   Entity 1: C::E name: body_3 , SW name: balance^escapement-1 ,  SW ref.type:2

link_7 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.03,-0.0256181804404151,-0.250816354295145)
cB = chrono.ChVectorD(-6.22881045830321e-05,-0.0256181804404151,-0.21783989029871)
dA = chrono.ChVectorD(0,-1,0)
dB = chrono.ChVectorD(0,-1,0)
link_7.Initialize(body_4,body_3,False,cA,cB,dB)
link_7.SetDistance(0)
link_7.SetName("Distanza3")
exported_items.append(link_7)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.03,-0.0256181804404151,-0.250816354295145)
dA = chrono.ChVectorD(0,-1,0)
cB = chrono.ChVectorD(-6.22881045830321e-05,-0.0256181804404151,-0.21783989029871)
dB = chrono.ChVectorD(0,-1,0)
link_8.Initialize(body_4,body_3,False,cA,cB,dA,dB)
link_8.SetName("Distanza3")
exported_items.append(link_8)


# Mate constraint: Concentrico3 [MateConcentric]
#   Entity 0: C::E name: body_2 , SW name: anchor^escapement-1/anchor_steel^anchor_escapement-1 ,  SW ref.type:2
#   Entity 1: C::E name: body_4 , SW name: truss^escapement-1 ,  SW ref.type:1

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.38777878078145e-17,-1.0842021724855e-19,-0.105)
dA = chrono.ChVectorD(0,-1,0)
cB = chrono.ChVectorD(0,-0.0256181804404151,-0.105)
dB = chrono.ChVectorD(0,-1,0)
link_9.Initialize(body_4,body_2,False,cB,cA,dB,dA)
link_9.SetName("Concentrico3")
exported_items.append(link_9)

link_10 = chrono.ChLinkMateGeneric()
link_10.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.38777878078145e-17,-1.0842021724855e-19,-0.105)
cB = chrono.ChVectorD(0,-0.0256181804404151,-0.105)
dA = chrono.ChVectorD(0,-1,0)
dB = chrono.ChVectorD(0,-1,0)
link_10.Initialize(body_4,body_2,False,cB,cA,dB,dA)
link_10.SetName("Concentrico3")
exported_items.append(link_10)


# Mate constraint: Distanza4 [MateDistanceDim]
#   Entity 0: C::E name: body_4 , SW name: truss^escapement-1 ,  SW ref.type:2
#   Entity 1: C::E name: body_2 , SW name: anchor^escapement-1/anchor_steel^anchor_escapement-1 ,  SW ref.type:2

link_11 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.03,-0.0127328264314352,-0.0874722553494342)
cB = chrono.ChVectorD(-0.013703234694226,-0.000732826431435213,-0.0789732577077453)
dA = chrono.ChVectorD(0,1,0)
dB = chrono.ChVectorD(0,-1,0)
link_11.Initialize(body_4,body_2,False,cA,cB,dB)
link_11.SetDistance(0.012)
link_11.SetName("Distanza4")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.03,-0.0127328264314352,-0.0874722553494342)
dA = chrono.ChVectorD(0,1,0)
cB = chrono.ChVectorD(-0.013703234694226,-0.000732826431435213,-0.0789732577077453)
dB = chrono.ChVectorD(0,-1,0)
link_12.SetFlipped(True)
link_12.Initialize(body_4,body_2,False,cA,cB,dA,dB)
link_12.SetName("Distanza4")
exported_items.append(link_12)

