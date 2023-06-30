# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 
# Assembly: C:\Users\SB\Documents\CAD\CAD_swiss_escapement\escapement.SLDASM


import pychrono as chrono 
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
body_1.SetName('anchor-1')
body_1.SetPos(chrono.ChVectorD(-0.0654391949504476,-0.0436913616481478,-0.187114016856068))
body_1.SetRot(chrono.ChQuaternionD(-0.330122510765304,-2.45537571233379e-16,0.943938095367494,-1.94130548328576e-15))
body_1.SetMass(0.27555084229585)
body_1.SetInertiaXX(chrono.ChVectorD(0.000519165190203935,0.000668606289866648,0.000157486745954595))
body_1.SetInertiaXY(chrono.ChVectorD(-1.52558132324693e-06,-4.37709719642171e-05,2.64531455782147e-07))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.00959089760382167,0.0486210957135911,-0.109837837967947),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChModelFileShape() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1.AddVisualShape(body_1_1_shape, chrono.ChFrameD(chrono.ChVectorD(0.0654391949504455,0.043691361648147,-0.187114016856069), chrono.ChQuaternionD(0.667465028254667,-0.66746502825467,-0.233431865984475,-0.233431865984475)))

# Visualization shape 
body_1_2_shape = chrono.ChModelFileShape() 
body_1_2_shape.SetFilename(shapes_dir +'body_1_2.obj') 
body_1.AddVisualShape(body_1_2_shape, chrono.ChFrameD(chrono.ChVectorD(0.0654391949504455,0.053691361648147,-0.187114016856069), chrono.ChQuaternionD(0.667465028254667,-0.66746502825467,-0.233431865984476,-0.233431865984475)))

# Collision material 
mat_1 = chrono.ChMaterialSurfaceNSC()

# Collision shapes 
body_1.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0.563277959889745; mr[1,0]=-3.62094996255649E-15; mr[2,0]=-0.826267474793996 
mr[0,1]=0.826267474793996; mr[1,1]=1.45159525356894E-15; mr[2,1]=0.563277959889746 
mr[0,2]=-8.40195363182309E-16; mr[1,2]=-1; mr[2,2]=3.80952479493291E-15 
body_1.GetCollisionModel().AddBox(mat_1, 0.005018, 0.004800, 0.005617,chrono.ChVectorD(-0.0422032375810062,0.0508827840639895,-0.0324552001423815),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.558382776027316; mr[1,0]=3.62946928396003E-15; mr[2,0]=0.829583434885261 
mr[0,1]=0.829583434885263; mr[1,1]=1.43016087497607E-15; mr[2,1]=0.558382776027314 
mr[0,2]=8.40195363182308E-16; mr[1,2]=1; mr[2,2]=-3.80952479493291E-15 
body_1.GetCollisionModel().AddBox(mat_1, 0.048767, 0.007000, 0.005617,chrono.ChVectorD(-0.0234341804765419,0.0508827840639893,-0.0701840838075975),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=0; mr[1,2]=1; mr[2,2]=0 
body_1.GetCollisionModel().AddCylinder(mat_1, 0.00152838632048226,0.003,chrono.ChVectorD(-0.0543540292757648,0.0415742064798313,-0.0242467886863428),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0.832851528781569; mr[1,0]=1.40880049479715E-15; mr[2,0]=0.553496459795547 
mr[0,1]=-0.55349645979555; mr[1,1]=3.63781370844921E-15; mr[2,1]=0.832851528781567 
mr[0,2]=-8.40195363182309E-16; mr[1,2]=-1; mr[2,2]=3.80952479493291E-15 
body_1.GetCollisionModel().AddBox(mat_1, 0.004800, 0.005018, 0.005617,chrono.ChVectorD(-0.0513228993817286,0.0508827840639894,-0.0385937622306218),mr)
pt_vect = chrono.vector_ChVectorD()
pt_vect.push_back(chrono.ChVectorD(0.0414088890638266,0.0416913616481475,-0.0742287544039014))
pt_vect.push_back(chrono.ChVectorD(0.0302575307494064,0.0416913616481475,-0.0786612714928038))
pt_vect.push_back(chrono.ChVectorD(0.0404169313636054,0.0416913616481474,-0.104220364494909))
pt_vect.push_back(chrono.ChVectorD(0.0512527702098685,0.0416913616481474,-0.0989940612628225))
pt_vect.push_back(chrono.ChVectorD(0.0414088890638266,0.0556913616481475,-0.0742287544039015))
pt_vect.push_back(chrono.ChVectorD(0.0302575307494064,0.0556913616481475,-0.0786612714928038))
pt_vect.push_back(chrono.ChVectorD(0.0404169313636054,0.0556913616481474,-0.104220364494909))
pt_vect.push_back(chrono.ChVectorD(0.0512527702098685,0.0556913616481474,-0.0989940612628226))
body_1.GetCollisionModel().AddConvexHull(mat_1, pt_vect)
pt_vect = chrono.vector_ChVectorD()
pt_vect.push_back(chrono.ChVectorD(-0.0253719200835922,0.0416913616481472,-0.150789419140605))
pt_vect.push_back(chrono.ChVectorD(-0.052662581388589,0.0416913616481473,-0.145818628764203))
pt_vect.push_back(chrono.ChVectorD(-0.0548129133033428,0.0416913616481472,-0.157624393146758))
pt_vect.push_back(chrono.ChVectorD(-0.0256253082894385,0.0416913616481472,-0.162940697669095))
pt_vect.push_back(chrono.ChVectorD(-0.0253719200835922,0.0556913616481472,-0.150789419140605))
pt_vect.push_back(chrono.ChVectorD(-0.052662581388589,0.0556913616481473,-0.145818628764203))
pt_vect.push_back(chrono.ChVectorD(-0.0548129133033428,0.0556913616481472,-0.157624393146758))
pt_vect.push_back(chrono.ChVectorD(-0.0256253082894385,0.0556913616481472,-0.162940697669095))
body_1.GetCollisionModel().AddConvexHull(mat_1, pt_vect)
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(True)

exported_items.append(body_1)



# Rigid body part
body_2= chrono.ChBodyAuxRef()
body_2.SetName('truss-1')
body_2.SetPos(chrono.ChVectorD(0,0,0))
body_2.SetRot(chrono.ChQuaternionD(0,0,0.707106781186548,0.707106781186547))
body_2.SetMass(1.02644190893078)
body_2.SetInertiaXX(chrono.ChVectorD(0.00421426808616481,0.00449296635132803,0.000334399259807225))
body_2.SetInertiaXY(chrono.ChVectorD(-1.30221244046362e-20,-1.60463939204644e-19,0.000147522492869351))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(5.96881493747061e-20,-0.120998995444052,-0.0214658323718929),chrono.ChQuaternionD(1,0,0,0)))
body_2.SetBodyFixed(True)

# Visualization shape 
body_2_1_shape = chrono.ChModelFileShape() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2.AddVisualShape(body_2_1_shape, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.ChQuaternionD(1,0,0,0)))


# Collision material 
mat_2 = chrono.ChMaterialSurfaceNSC()

# Collision shapes 
body_2.GetCollisionModel().ClearModel()
body_2.GetCollisionModel().AddCylinder(mat_2, 0.00307962367981624,0.024,chrono.ChVectorD(-0.0119269719437114,-0.166883602750055,-0.000732826431435216),chrono.ChMatrix33D(1))
body_2.GetCollisionModel().AddCylinder(mat_2, 0.00307962367981624,0.024,chrono.ChVectorD(0.0119269719437114,-0.166883602750055,-0.000732826431435216),chrono.ChMatrix33D(1))
body_2.GetCollisionModel().BuildModel()
body_2.SetCollide(True)

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRef()
body_3.SetName('balance-1')
body_3.SetPos(chrono.ChVectorD(-0.00490107217461526,3.57656060958299e-05,-5.48348916194744e-05))
body_3.SetRot(chrono.ChQuaternionD(0.714973388107422,0,0.699151667593086,0))
body_3.SetMass(1.81117045126479)
body_3.SetInertiaXX(chrono.ChVectorD(0.0202820042137968,0.0412119638526683,0.0212249765673293))
body_3.SetInertiaXY(chrono.ChVectorD(-6.70222328430828e-08,8.32022499599561e-05,5.4837968152323e-07))
body_3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.21901864850551,0.0330968348592532,9.59245261293635e-07),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChModelFileShape() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3.AddVisualShape(body_3_1_shape, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.ChQuaternionD(1,0,0,0)))


# Collision material 
mat_3 = chrono.ChMaterialSurfaceNSC()

# Collision shapes 
body_3.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0.973704418632404; mr[1,0]=0; mr[2,0]=0.227815067841729 
mr[0,1]=-0.227815067841728; mr[1,1]=0; mr[2,1]=0.973704418632405 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_3.GetCollisionModel().AddBox(mat_3, 0.001161, 0.005109, 0.015000,chrono.ChVectorD(0.190889475660623,0.0103234187261071,-0.00657694561809265),mr)
body_3.GetCollisionModel().BuildModel()
body_3.SetCollide(True)

exported_items.append(body_3)



# Rigid body part
body_4= chrono.ChBodyAuxRef()
body_4.SetName('escape_wheel-1')
body_4.SetPos(chrono.ChVectorD(0,0,0))
body_4.SetRot(chrono.ChQuaternionD(0.580317762346015,-0.580317762346015,0.404018928647798,0.404018928647798))
body_4.SetMass(0.384487702285899)
body_4.SetInertiaXX(chrono.ChVectorD(0.0010834203298181,0.000678649729047534,0.000614415177644497))
body_4.SetInertiaXY(chrono.ChVectorD(0.000173569396944232,-3.15578528386675e-19,-8.08426348941013e-19))
body_4.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-1.55722279465587e-17,3.80579019057423e-17,0.00631453892912046),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChModelFileShape() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4.AddVisualShape(body_4_1_shape, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.ChQuaternionD(1,0,0,0)))


# Collision material 
mat_4 = chrono.ChMaterialSurfaceNSC()

# Collision shapes 
body_4.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=-0.395048452878596; mr[1,1]=0.918660285349393; mr[2,1]=0 
mr[0,2]=0.918660285349393; mr[1,2]=0.395048452878596; mr[2,2]=0 
body_4.GetCollisionModel().AddBox(mat_4, 0.005000, 0.006561, 0.004000,chrono.ChVectorD(-0.0579385872259149,-0.0683311581039222,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=0.0127580809140941; mr[1,1]=0.999918612373722; mr[2,1]=0 
mr[0,2]=0.999918612373722; mr[1,2]=-0.0127580809140941; mr[2,2]=0 
body_4.GetCollisionModel().AddBox(mat_4, 0.005000, 0.006561, 0.004000,chrono.ChVectorD(-0.0807223190471352,-0.0388578726284734,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=0.418358626613207; mr[1,1]=0.908281927343218; mr[2,1]=0 
mr[0,2]=0.908281927343218; mr[1,2]=-0.418358626613207; mr[2,2]=0 
body_4.GetCollisionModel().AddBox(mat_4, 0.005000, 0.006561, 0.004000,chrono.ChVectorD(-0.0895484285658595,-0.00266570796287103,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=-0.947036648244772; mr[1,1]=0.321125500204061; mr[2,1]=0 
mr[0,2]=0.321125500204061; mr[1,2]=0.947036648244772; mr[2,2]=0 
body_4.GetCollisionModel().AddBox(mat_4, 0.005000, 0.006561, 0.004000,chrono.ChVectorD(0.0120114645617556,-0.0887792305476897,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=0.75162116510209; mr[1,1]=0.659595045592807; mr[2,1]=0 
mr[0,2]=0.659595045592807; mr[1,2]=-0.75162116510209; mr[2,2]=0 
body_4.GetCollisionModel().AddBox(mat_4, 0.005000, 0.006561, 0.004000,chrono.ChVectorD(-0.0828908012636124,0.0339873818267083,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=0.954921575880902; mr[1,1]=0.296858188226525; mr[2,1]=0 
mr[0,2]=0.296858188226525; mr[1,2]=-0.954921575880902; mr[2,2]=0 
body_4.GetCollisionModel().AddBox(mat_4, 0.005000, 0.006561, 0.004000,chrono.ChVectorD(-0.0619006013835978,0.0647637445327791,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=0.993107370999733; mr[1,1]=-0.117208146756099; mr[2,1]=0 
mr[0,2]=-0.117208146756099; mr[1,2]=-0.993107370999733; mr[2,2]=0 
body_4.GetCollisionModel().AddBox(mat_4, 0.005000, 0.006561, 0.004000,chrono.ChVectorD(-0.0302072251750497,0.084341867448984,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=0.85957587957548; mr[1,1]=-0.511008128362006; mr[2,1]=0 
mr[0,2]=-0.511008128362006; mr[1,2]=-0.85957587957548; mr[2,2]=0 
body_4.GetCollisionModel().AddBox(mat_4, 0.005000, 0.006561, 0.004000,chrono.ChVectorD(0.00670925469029009,0.0893365152614482,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=0.577415909570911; mr[1,1]=-0.816450162211018; mr[2,1]=0 
mr[0,2]=-0.816450162211018; mr[1,2]=-0.577415909570911; mr[2,2]=0 
body_4.GetCollisionModel().AddBox(mat_4, 0.005000, 0.006561, 0.004000,chrono.ChVectorD(0.0424656434680133,0.0788840679884458,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=0.195415483142673; mr[1,1]=-0.980720545796873; mr[2,1]=0 
mr[0,2]=-0.980720545796873; mr[1,2]=-0.195415483142673; mr[2,2]=0 
body_4.GetCollisionModel().AddBox(mat_4, 0.005000, 0.006561, 0.004000,chrono.ChVectorD(0.0708793367018574,0.0547918487209811,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=-0.220374055614862; mr[1,1]=-0.975415437447992; mr[2,1]=0 
mr[0,2]=-0.975415437447992; mr[1,2]=0.220374055614862; mr[2,2]=0 
body_4.GetCollisionModel().AddBox(mat_4, 0.005000, 0.006561, 0.004000,chrono.ChVectorD(0.0870373487013913,0.0212256210413399,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=-0.598058918121146; mr[1,1]=-0.801452138593294; mr[2,1]=0 
mr[0,2]=-0.801452138593294; mr[1,2]=0.598058918121146; mr[2,2]=0 
body_4.GetCollisionModel().AddBox(mat_4, 0.005000, 0.006561, 0.004000,chrono.ChVectorD(0.0881458124009648,-0.0160107093450625,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=-0.872333960489579; mr[1,1]=-0.48891048401171; mr[2,1]=0 
mr[0,2]=-0.48891048401171; mr[1,2]=0.872333960489579; mr[2,2]=0 
body_4.GetCollisionModel().AddBox(mat_4, 0.005000, 0.006561, 0.004000,chrono.ChVectorD(0.0740130643568451,-0.0504786426329755,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=-0.995774536184123; mr[1,1]=-0.091831765132195; mr[2,1]=0 
mr[0,2]=-0.091831765132195; mr[1,2]=0.995774536184123; mr[2,2]=0 
body_4.GetCollisionModel().AddBox(mat_4, 0.005000, 0.006561, 0.004000,chrono.ChVectorD(0.0470827850978458,-0.0762183600255753,0.0025),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=-0.734547520266048; mr[1,1]=0.678557249221464; mr[2,1]=0 
mr[0,2]=0.678557249221464; mr[1,2]=0.734547520266048; mr[2,2]=0 
body_4.GetCollisionModel().AddBox(mat_4, 0.005000, 0.006561, 0.004000,chrono.ChVectorD(-0.025136747317793,-0.0859893655741196,0.0025),mr)
body_4.GetCollisionModel().BuildModel()
body_4.SetCollide(True)

exported_items.append(body_4)




# Mate constraint: Concentrico1 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: escape_wheel-1 ,  SW ref.type:1 (1)
#   Entity 1: C::E name: body_2 , SW name: truss-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0,-0.03,0)
dA = chrono.ChVectorD(0,-1,0)
cB = chrono.ChVectorD(0,-0.03,0)
dB = chrono.ChVectorD(0,1,0)
link_1.SetFlipped(True)
link_1.Initialize(body_4,body_2,False,cA,cB,dA,dB)
link_1.SetName("Concentrico1")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0,-0.03,0)
cB = chrono.ChVectorD(0,-0.03,0)
dA = chrono.ChVectorD(0,-1,0)
dB = chrono.ChVectorD(0,1,0)
link_2.Initialize(body_4,body_2,False,cA,cB,dA,dB)
link_2.SetName("Concentrico1")
exported_items.append(link_2)


# Mate constraint: Concentrico2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: balance-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: truss-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-1.47451495458029e-16,3.57779056074965e-05,-0.219053185080869)
dA = chrono.ChVectorD(0,-1,0)
cB = chrono.ChVectorD(0,-0.03,-0.21905318508087)
dB = chrono.ChVectorD(0,1,0)
link_3.SetFlipped(True)
link_3.Initialize(body_3,body_2,False,cA,cB,dA,dB)
link_3.SetName("Concentrico2")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateGeneric()
link_4.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-1.47451495458029e-16,3.57779056074965e-05,-0.219053185080869)
cB = chrono.ChVectorD(0,-0.03,-0.21905318508087)
dA = chrono.ChVectorD(0,-1,0)
dB = chrono.ChVectorD(0,1,0)
link_4.Initialize(body_3,body_2,False,cA,cB,dA,dB)
link_4.SetName("Concentrico2")
exported_items.append(link_4)


# Mate constraint: Concentrico3 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_1 , SW name: anchor-1/anchor_steel-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: truss-1 ,  SW ref.type:1 (1)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-5.82867087928207e-18,-5.63785129692462e-18,-0.105)
dA = chrono.ChVectorD(2.8134962801131e-17,-1,2.10255524402378e-15)
cB = chrono.ChVectorD(0,-0.0256181804404151,-0.105)
dB = chrono.ChVectorD(0,-1,0)
link_5.Initialize(body_2,body_1,False,cB,cA,dB,dA)
link_5.SetName("Concentrico3")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateGeneric()
link_6.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-5.82867087928207e-18,-5.63785129692462e-18,-0.105)
cB = chrono.ChVectorD(0,-0.0256181804404151,-0.105)
dA = chrono.ChVectorD(2.8134962801131e-17,-1,2.10255524402378e-15)
dB = chrono.ChVectorD(0,-1,0)
link_6.Initialize(body_2,body_1,False,cB,cA,dB,dA)
link_6.SetName("Concentrico3")
exported_items.append(link_6)


# Mate constraint: Coincident1 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_4 , SW name: escape_wheel-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: truss-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.00188591368619475,-0.03,0.000697938817270251)
cB = chrono.ChVectorD(0.03,-0.03,-0.0810582164392656)
dA = chrono.ChVectorD(0,-1,0)
dB = chrono.ChVectorD(0,-1,0)
link_7.Initialize(body_4,body_2,False,cA,cB,dB)
link_7.SetDistance(0)
link_7.SetName("Coincident1")
exported_items.append(link_7)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.00188591368619475,-0.03,0.000697938817270251)
dA = chrono.ChVectorD(0,-1,0)
cB = chrono.ChVectorD(0.03,-0.03,-0.0810582164392656)
dB = chrono.ChVectorD(0,-1,0)
link_8.Initialize(body_4,body_2,False,cA,cB,dA,dB)
link_8.SetName("Coincident1")
exported_items.append(link_8)


# Mate constraint: Coincident2 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_3 , SW name: balance-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: truss-1 ,  SW ref.type:2 (2)

link_9 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-2.71818751627004e-05,-0.0256181804404151,-0.217838596598275)
cB = chrono.ChVectorD(0.03,-0.0256181804404151,-0.250816354295145)
dA = chrono.ChVectorD(0,-1,0)
dB = chrono.ChVectorD(0,-1,0)
link_9.Initialize(body_3,body_2,False,cA,cB,dB)
link_9.SetDistance(0)
link_9.SetName("Coincident2")
exported_items.append(link_9)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-2.71818751627004e-05,-0.0256181804404151,-0.217838596598275)
dA = chrono.ChVectorD(0,-1,0)
cB = chrono.ChVectorD(0.03,-0.0256181804404151,-0.250816354295145)
dB = chrono.ChVectorD(0,-1,0)
link_10.Initialize(body_3,body_2,False,cA,cB,dA,dB)
link_10.SetName("Coincident2")
exported_items.append(link_10)


# Mate constraint: Coincident5 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: anchor-1/anchor_steel-1 ,  SW ref.type:1 (1)
#   Entity 1: C::E name: body_2 , SW name: truss-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-5.47043328128038e-18,-0.0127328264314352,-0.105)
cB = chrono.ChVectorD(0.03,-0.0127328264314352,-0.0874722553494342)
dA = chrono.ChVectorD(2.8134962801131e-17,-1,2.10255524402378e-15)
dB = chrono.ChVectorD(0,1,0)
link_11.Initialize(body_1,body_2,False,cA,cB,dB)
link_11.SetDistance(0)
link_11.SetName("Coincident5")
exported_items.append(link_11)


# ChLinkMateOrthogonal skipped because directions not orthogonal! 
