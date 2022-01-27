/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//cbtShapeHull was implemented by John McCutchan.

#include "cbtShapeHull.h"
#include "LinearMath/cbtConvexHull.h"

#define NUM_UNITSPHERE_POINTS 42
#define NUM_UNITSPHERE_POINTS_HIGHRES 256

cbtShapeHull::cbtShapeHull(const cbtConvexShape* shape)
{
	m_shape = shape;
	m_vertices.clear();
	m_indices.clear();
	m_numIndices = 0;
}

cbtShapeHull::~cbtShapeHull()
{
	m_indices.clear();
	m_vertices.clear();
}

bool cbtShapeHull::buildHull(cbtScalar /*margin*/, int highres)
{
	
	int numSampleDirections = highres ? NUM_UNITSPHERE_POINTS_HIGHRES : NUM_UNITSPHERE_POINTS;
	cbtVector3 supportPoints[NUM_UNITSPHERE_POINTS_HIGHRES + MAX_PREFERRED_PENETRATION_DIRECTIONS * 2];
	int i;
	for (i = 0; i < numSampleDirections; i++)
	{
		supportPoints[i] = m_shape->localGetSupportingVertex(getUnitSpherePoints(highres)[i]);
	}

	int numPDA = m_shape->getNumPreferredPenetrationDirections();
	if (numPDA)
	{
		for (int s = 0; s < numPDA; s++)
		{
			cbtVector3 norm;
			m_shape->getPreferredPenetrationDirection(s, norm);
			supportPoints[i++] = m_shape->localGetSupportingVertex(norm);
			numSampleDirections++;
		}
	}
	HullDesc hd;
	hd.mFlags = QF_TRIANGLES;
	hd.mVcount = static_cast<unsigned int>(numSampleDirections);

#ifdef BT_USE_DOUBLE_PRECISION
	hd.mVertices = &supportPoints[0];
	hd.mVertexStride = sizeof(cbtVector3);
#else
	hd.mVertices = &supportPoints[0];
	hd.mVertexStride = sizeof(cbtVector3);
#endif

	HullLibrary hl;
	HullResult hr;
	if (hl.CreateConvexHull(hd, hr) == QE_FAIL)
	{
		return false;
	}

	m_vertices.resize(static_cast<int>(hr.mNumOutputVertices));

	for (i = 0; i < static_cast<int>(hr.mNumOutputVertices); i++)
	{
		m_vertices[i] = hr.m_OutputVertices[i];
	}
	m_numIndices = hr.mNumIndices;
	m_indices.resize(static_cast<int>(m_numIndices));
	for (i = 0; i < static_cast<int>(m_numIndices); i++)
	{
		m_indices[i] = hr.m_Indices[i];
	}

	// free temporary hull result that we just copied
	hl.ReleaseResult(hr);

	return true;
}

int cbtShapeHull::numTriangles() const
{
	return static_cast<int>(m_numIndices / 3);
}

int cbtShapeHull::numVertices() const
{
	return m_vertices.size();
}

int cbtShapeHull::numIndices() const
{
	return static_cast<int>(m_numIndices);
}

cbtVector3* cbtShapeHull::getUnitSpherePoints(int highres)
{
	static cbtVector3 sUnitSpherePointsHighres[NUM_UNITSPHERE_POINTS_HIGHRES + MAX_PREFERRED_PENETRATION_DIRECTIONS * 2] =
		{
			cbtVector3(cbtScalar(0.997604), cbtScalar(0.067004), cbtScalar(0.017144)),
			cbtVector3(cbtScalar(0.984139), cbtScalar(-0.086784), cbtScalar(-0.154427)),
			cbtVector3(cbtScalar(0.971065), cbtScalar(0.124164), cbtScalar(-0.203224)),
			cbtVector3(cbtScalar(0.955844), cbtScalar(0.291173), cbtScalar(-0.037704)),
			cbtVector3(cbtScalar(0.957405), cbtScalar(0.212238), cbtScalar(0.195157)),
			cbtVector3(cbtScalar(0.971650), cbtScalar(-0.012709), cbtScalar(0.235561)),
			cbtVector3(cbtScalar(0.984920), cbtScalar(-0.161831), cbtScalar(0.059695)),
			cbtVector3(cbtScalar(0.946673), cbtScalar(-0.299288), cbtScalar(-0.117536)),
			cbtVector3(cbtScalar(0.922670), cbtScalar(-0.219186), cbtScalar(-0.317019)),
			cbtVector3(cbtScalar(0.928134), cbtScalar(-0.007265), cbtScalar(-0.371867)),
			cbtVector3(cbtScalar(0.875642), cbtScalar(0.198434), cbtScalar(-0.439988)),
			cbtVector3(cbtScalar(0.908035), cbtScalar(0.325975), cbtScalar(-0.262562)),
			cbtVector3(cbtScalar(0.864519), cbtScalar(0.488706), cbtScalar(-0.116755)),
			cbtVector3(cbtScalar(0.893009), cbtScalar(0.428046), cbtScalar(0.137185)),
			cbtVector3(cbtScalar(0.857494), cbtScalar(0.362137), cbtScalar(0.364776)),
			cbtVector3(cbtScalar(0.900815), cbtScalar(0.132524), cbtScalar(0.412987)),
			cbtVector3(cbtScalar(0.934964), cbtScalar(-0.241739), cbtScalar(0.259179)),
			cbtVector3(cbtScalar(0.894570), cbtScalar(-0.103504), cbtScalar(0.434263)),
			cbtVector3(cbtScalar(0.922085), cbtScalar(-0.376668), cbtScalar(0.086241)),
			cbtVector3(cbtScalar(0.862177), cbtScalar(-0.499154), cbtScalar(-0.085330)),
			cbtVector3(cbtScalar(0.861982), cbtScalar(-0.420218), cbtScalar(-0.282861)),
			cbtVector3(cbtScalar(0.818076), cbtScalar(-0.328256), cbtScalar(-0.471804)),
			cbtVector3(cbtScalar(0.762657), cbtScalar(-0.179329), cbtScalar(-0.621124)),
			cbtVector3(cbtScalar(0.826857), cbtScalar(0.019760), cbtScalar(-0.561786)),
			cbtVector3(cbtScalar(0.731434), cbtScalar(0.206599), cbtScalar(-0.649817)),
			cbtVector3(cbtScalar(0.769486), cbtScalar(0.379052), cbtScalar(-0.513770)),
			cbtVector3(cbtScalar(0.796806), cbtScalar(0.507176), cbtScalar(-0.328145)),
			cbtVector3(cbtScalar(0.679722), cbtScalar(0.684101), cbtScalar(-0.264123)),
			cbtVector3(cbtScalar(0.786854), cbtScalar(0.614886), cbtScalar(0.050912)),
			cbtVector3(cbtScalar(0.769486), cbtScalar(0.571141), cbtScalar(0.285139)),
			cbtVector3(cbtScalar(0.707432), cbtScalar(0.492789), cbtScalar(0.506288)),
			cbtVector3(cbtScalar(0.774560), cbtScalar(0.268037), cbtScalar(0.572652)),
			cbtVector3(cbtScalar(0.796220), cbtScalar(0.031230), cbtScalar(0.604077)),
			cbtVector3(cbtScalar(0.837395), cbtScalar(-0.320285), cbtScalar(0.442461)),
			cbtVector3(cbtScalar(0.848127), cbtScalar(-0.450548), cbtScalar(0.278307)),
			cbtVector3(cbtScalar(0.775536), cbtScalar(-0.206354), cbtScalar(0.596465)),
			cbtVector3(cbtScalar(0.816320), cbtScalar(-0.567007), cbtScalar(0.109469)),
			cbtVector3(cbtScalar(0.741191), cbtScalar(-0.668690), cbtScalar(-0.056832)),
			cbtVector3(cbtScalar(0.755632), cbtScalar(-0.602975), cbtScalar(-0.254949)),
			cbtVector3(cbtScalar(0.720311), cbtScalar(-0.521318), cbtScalar(-0.457165)),
			cbtVector3(cbtScalar(0.670746), cbtScalar(-0.386583), cbtScalar(-0.632835)),
			cbtVector3(cbtScalar(0.587031), cbtScalar(-0.219769), cbtScalar(-0.778836)),
			cbtVector3(cbtScalar(0.676015), cbtScalar(-0.003182), cbtScalar(-0.736676)),
			cbtVector3(cbtScalar(0.566932), cbtScalar(0.186963), cbtScalar(-0.802064)),
			cbtVector3(cbtScalar(0.618254), cbtScalar(0.398105), cbtScalar(-0.677533)),
			cbtVector3(cbtScalar(0.653964), cbtScalar(0.575224), cbtScalar(-0.490933)),
			cbtVector3(cbtScalar(0.525367), cbtScalar(0.743205), cbtScalar(-0.414028)),
			cbtVector3(cbtScalar(0.506439), cbtScalar(0.836528), cbtScalar(-0.208885)),
			cbtVector3(cbtScalar(0.651427), cbtScalar(0.756426), cbtScalar(-0.056247)),
			cbtVector3(cbtScalar(0.641670), cbtScalar(0.745149), cbtScalar(0.180908)),
			cbtVector3(cbtScalar(0.602643), cbtScalar(0.687211), cbtScalar(0.405180)),
			cbtVector3(cbtScalar(0.516586), cbtScalar(0.596999), cbtScalar(0.613447)),
			cbtVector3(cbtScalar(0.602252), cbtScalar(0.387801), cbtScalar(0.697573)),
			cbtVector3(cbtScalar(0.646549), cbtScalar(0.153911), cbtScalar(0.746956)),
			cbtVector3(cbtScalar(0.650842), cbtScalar(-0.087756), cbtScalar(0.753983)),
			cbtVector3(cbtScalar(0.740411), cbtScalar(-0.497404), cbtScalar(0.451830)),
			cbtVector3(cbtScalar(0.726946), cbtScalar(-0.619890), cbtScalar(0.295093)),
			cbtVector3(cbtScalar(0.637768), cbtScalar(-0.313092), cbtScalar(0.703624)),
			cbtVector3(cbtScalar(0.678942), cbtScalar(-0.722934), cbtScalar(0.126645)),
			cbtVector3(cbtScalar(0.489072), cbtScalar(-0.867195), cbtScalar(-0.092942)),
			cbtVector3(cbtScalar(0.622742), cbtScalar(-0.757541), cbtScalar(-0.194636)),
			cbtVector3(cbtScalar(0.596788), cbtScalar(-0.693576), cbtScalar(-0.403098)),
			cbtVector3(cbtScalar(0.550150), cbtScalar(-0.582172), cbtScalar(-0.598287)),
			cbtVector3(cbtScalar(0.474436), cbtScalar(-0.429745), cbtScalar(-0.768101)),
			cbtVector3(cbtScalar(0.372574), cbtScalar(-0.246016), cbtScalar(-0.894583)),
			cbtVector3(cbtScalar(0.480095), cbtScalar(-0.026513), cbtScalar(-0.876626)),
			cbtVector3(cbtScalar(0.352474), cbtScalar(0.177242), cbtScalar(-0.918787)),
			cbtVector3(cbtScalar(0.441848), cbtScalar(0.374386), cbtScalar(-0.814946)),
			cbtVector3(cbtScalar(0.492389), cbtScalar(0.582223), cbtScalar(-0.646693)),
			cbtVector3(cbtScalar(0.343498), cbtScalar(0.866080), cbtScalar(-0.362693)),
			cbtVector3(cbtScalar(0.362036), cbtScalar(0.745149), cbtScalar(-0.559639)),
			cbtVector3(cbtScalar(0.334131), cbtScalar(0.937044), cbtScalar(-0.099774)),
			cbtVector3(cbtScalar(0.486925), cbtScalar(0.871718), cbtScalar(0.052473)),
			cbtVector3(cbtScalar(0.452776), cbtScalar(0.845665), cbtScalar(0.281820)),
			cbtVector3(cbtScalar(0.399503), cbtScalar(0.771785), cbtScalar(0.494576)),
			cbtVector3(cbtScalar(0.296469), cbtScalar(0.673018), cbtScalar(0.677469)),
			cbtVector3(cbtScalar(0.392088), cbtScalar(0.479179), cbtScalar(0.785213)),
			cbtVector3(cbtScalar(0.452190), cbtScalar(0.252094), cbtScalar(0.855286)),
			cbtVector3(cbtScalar(0.478339), cbtScalar(0.013149), cbtScalar(0.877928)),
			cbtVector3(cbtScalar(0.481656), cbtScalar(-0.219380), cbtScalar(0.848259)),
			cbtVector3(cbtScalar(0.615327), cbtScalar(-0.494293), cbtScalar(0.613837)),
			cbtVector3(cbtScalar(0.594642), cbtScalar(-0.650414), cbtScalar(0.472325)),
			cbtVector3(cbtScalar(0.562249), cbtScalar(-0.771345), cbtScalar(0.297631)),
			cbtVector3(cbtScalar(0.467411), cbtScalar(-0.437133), cbtScalar(0.768231)),
			cbtVector3(cbtScalar(0.519513), cbtScalar(-0.847947), cbtScalar(0.103808)),
			cbtVector3(cbtScalar(0.297640), cbtScalar(-0.938159), cbtScalar(-0.176288)),
			cbtVector3(cbtScalar(0.446727), cbtScalar(-0.838615), cbtScalar(-0.311359)),
			cbtVector3(cbtScalar(0.331790), cbtScalar(-0.942437), cbtScalar(0.040762)),
			cbtVector3(cbtScalar(0.413358), cbtScalar(-0.748403), cbtScalar(-0.518259)),
			cbtVector3(cbtScalar(0.347596), cbtScalar(-0.621640), cbtScalar(-0.701737)),
			cbtVector3(cbtScalar(0.249831), cbtScalar(-0.456186), cbtScalar(-0.853984)),
			cbtVector3(cbtScalar(0.131772), cbtScalar(-0.262931), cbtScalar(-0.955678)),
			cbtVector3(cbtScalar(0.247099), cbtScalar(-0.042261), cbtScalar(-0.967975)),
			cbtVector3(cbtScalar(0.113624), cbtScalar(0.165965), cbtScalar(-0.979491)),
			cbtVector3(cbtScalar(0.217438), cbtScalar(0.374580), cbtScalar(-0.901220)),
			cbtVector3(cbtScalar(0.307983), cbtScalar(0.554615), cbtScalar(-0.772786)),
			cbtVector3(cbtScalar(0.166702), cbtScalar(0.953181), cbtScalar(-0.252021)),
			cbtVector3(cbtScalar(0.172751), cbtScalar(0.844499), cbtScalar(-0.506743)),
			cbtVector3(cbtScalar(0.177630), cbtScalar(0.711125), cbtScalar(-0.679876)),
			cbtVector3(cbtScalar(0.120064), cbtScalar(0.992260), cbtScalar(-0.030482)),
			cbtVector3(cbtScalar(0.289640), cbtScalar(0.949098), cbtScalar(0.122546)),
			cbtVector3(cbtScalar(0.239879), cbtScalar(0.909047), cbtScalar(0.340377)),
			cbtVector3(cbtScalar(0.181142), cbtScalar(0.821363), cbtScalar(0.540641)),
			cbtVector3(cbtScalar(0.066986), cbtScalar(0.719097), cbtScalar(0.691327)),
			cbtVector3(cbtScalar(0.156750), cbtScalar(0.545478), cbtScalar(0.823079)),
			cbtVector3(cbtScalar(0.236172), cbtScalar(0.342306), cbtScalar(0.909353)),
			cbtVector3(cbtScalar(0.277541), cbtScalar(0.112693), cbtScalar(0.953856)),
			cbtVector3(cbtScalar(0.295299), cbtScalar(-0.121974), cbtScalar(0.947415)),
			cbtVector3(cbtScalar(0.287883), cbtScalar(-0.349254), cbtScalar(0.891591)),
			cbtVector3(cbtScalar(0.437165), cbtScalar(-0.634666), cbtScalar(0.636869)),
			cbtVector3(cbtScalar(0.407113), cbtScalar(-0.784954), cbtScalar(0.466664)),
			cbtVector3(cbtScalar(0.375111), cbtScalar(-0.888193), cbtScalar(0.264839)),
			cbtVector3(cbtScalar(0.275394), cbtScalar(-0.560591), cbtScalar(0.780723)),
			cbtVector3(cbtScalar(0.122015), cbtScalar(-0.992209), cbtScalar(-0.024821)),
			cbtVector3(cbtScalar(0.087866), cbtScalar(-0.966156), cbtScalar(-0.241676)),
			cbtVector3(cbtScalar(0.239489), cbtScalar(-0.885665), cbtScalar(-0.397437)),
			cbtVector3(cbtScalar(0.167287), cbtScalar(-0.965184), cbtScalar(0.200817)),
			cbtVector3(cbtScalar(0.201632), cbtScalar(-0.776789), cbtScalar(-0.596335)),
			cbtVector3(cbtScalar(0.122015), cbtScalar(-0.637971), cbtScalar(-0.760098)),
			cbtVector3(cbtScalar(0.008054), cbtScalar(-0.464741), cbtScalar(-0.885214)),
			cbtVector3(cbtScalar(-0.116054), cbtScalar(-0.271096), cbtScalar(-0.955482)),
			cbtVector3(cbtScalar(-0.000727), cbtScalar(-0.056065), cbtScalar(-0.998424)),
			cbtVector3(cbtScalar(-0.134007), cbtScalar(0.152939), cbtScalar(-0.978905)),
			cbtVector3(cbtScalar(-0.025900), cbtScalar(0.366026), cbtScalar(-0.930108)),
			cbtVector3(cbtScalar(0.081231), cbtScalar(0.557337), cbtScalar(-0.826072)),
			cbtVector3(cbtScalar(-0.002874), cbtScalar(0.917213), cbtScalar(-0.398023)),
			cbtVector3(cbtScalar(-0.050683), cbtScalar(0.981761), cbtScalar(-0.182534)),
			cbtVector3(cbtScalar(-0.040536), cbtScalar(0.710153), cbtScalar(-0.702713)),
			cbtVector3(cbtScalar(-0.139081), cbtScalar(0.827973), cbtScalar(-0.543048)),
			cbtVector3(cbtScalar(-0.101029), cbtScalar(0.994010), cbtScalar(0.041152)),
			cbtVector3(cbtScalar(0.069328), cbtScalar(0.978067), cbtScalar(0.196133)),
			cbtVector3(cbtScalar(0.023860), cbtScalar(0.911380), cbtScalar(0.410645)),
			cbtVector3(cbtScalar(-0.153521), cbtScalar(0.736789), cbtScalar(0.658145)),
			cbtVector3(cbtScalar(-0.070002), cbtScalar(0.591750), cbtScalar(0.802780)),
			cbtVector3(cbtScalar(0.002590), cbtScalar(0.312948), cbtScalar(0.949562)),
			cbtVector3(cbtScalar(0.090988), cbtScalar(-0.020680), cbtScalar(0.995627)),
			cbtVector3(cbtScalar(0.088842), cbtScalar(-0.250099), cbtScalar(0.964006)),
			cbtVector3(cbtScalar(0.083378), cbtScalar(-0.470185), cbtScalar(0.878318)),
			cbtVector3(cbtScalar(0.240074), cbtScalar(-0.749764), cbtScalar(0.616374)),
			cbtVector3(cbtScalar(0.210803), cbtScalar(-0.885860), cbtScalar(0.412987)),
			cbtVector3(cbtScalar(0.077524), cbtScalar(-0.660524), cbtScalar(0.746565)),
			cbtVector3(cbtScalar(-0.096736), cbtScalar(-0.990070), cbtScalar(-0.100945)),
			cbtVector3(cbtScalar(-0.052634), cbtScalar(-0.990264), cbtScalar(0.127426)),
			cbtVector3(cbtScalar(-0.106102), cbtScalar(-0.938354), cbtScalar(-0.328340)),
			cbtVector3(cbtScalar(0.013323), cbtScalar(-0.863112), cbtScalar(-0.504596)),
			cbtVector3(cbtScalar(-0.002093), cbtScalar(-0.936993), cbtScalar(0.349161)),
			cbtVector3(cbtScalar(-0.106297), cbtScalar(-0.636610), cbtScalar(-0.763612)),
			cbtVector3(cbtScalar(-0.229430), cbtScalar(-0.463769), cbtScalar(-0.855546)),
			cbtVector3(cbtScalar(-0.245236), cbtScalar(-0.066175), cbtScalar(-0.966999)),
			cbtVector3(cbtScalar(-0.351587), cbtScalar(-0.270513), cbtScalar(-0.896145)),
			cbtVector3(cbtScalar(-0.370906), cbtScalar(0.133108), cbtScalar(-0.918982)),
			cbtVector3(cbtScalar(-0.264360), cbtScalar(0.346000), cbtScalar(-0.900049)),
			cbtVector3(cbtScalar(-0.151375), cbtScalar(0.543728), cbtScalar(-0.825291)),
			cbtVector3(cbtScalar(-0.218697), cbtScalar(0.912741), cbtScalar(-0.344346)),
			cbtVector3(cbtScalar(-0.274507), cbtScalar(0.953764), cbtScalar(-0.121635)),
			cbtVector3(cbtScalar(-0.259677), cbtScalar(0.692266), cbtScalar(-0.673044)),
			cbtVector3(cbtScalar(-0.350416), cbtScalar(0.798810), cbtScalar(-0.488786)),
			cbtVector3(cbtScalar(-0.320170), cbtScalar(0.941127), cbtScalar(0.108297)),
			cbtVector3(cbtScalar(-0.147667), cbtScalar(0.952792), cbtScalar(0.265034)),
			cbtVector3(cbtScalar(-0.188061), cbtScalar(0.860636), cbtScalar(0.472910)),
			cbtVector3(cbtScalar(-0.370906), cbtScalar(0.739900), cbtScalar(0.560941)),
			cbtVector3(cbtScalar(-0.297143), cbtScalar(0.585334), cbtScalar(0.754178)),
			cbtVector3(cbtScalar(-0.189622), cbtScalar(0.428241), cbtScalar(0.883393)),
			cbtVector3(cbtScalar(-0.091272), cbtScalar(0.098695), cbtScalar(0.990747)),
			cbtVector3(cbtScalar(-0.256945), cbtScalar(0.228375), cbtScalar(0.938827)),
			cbtVector3(cbtScalar(-0.111761), cbtScalar(-0.133251), cbtScalar(0.984696)),
			cbtVector3(cbtScalar(-0.118006), cbtScalar(-0.356253), cbtScalar(0.926725)),
			cbtVector3(cbtScalar(-0.119372), cbtScalar(-0.563896), cbtScalar(0.817029)),
			cbtVector3(cbtScalar(0.041228), cbtScalar(-0.833949), cbtScalar(0.550010)),
			cbtVector3(cbtScalar(-0.121909), cbtScalar(-0.736543), cbtScalar(0.665172)),
			cbtVector3(cbtScalar(-0.307681), cbtScalar(-0.931160), cbtScalar(-0.195026)),
			cbtVector3(cbtScalar(-0.283679), cbtScalar(-0.957990), cbtScalar(0.041348)),
			cbtVector3(cbtScalar(-0.227284), cbtScalar(-0.935243), cbtScalar(0.270890)),
			cbtVector3(cbtScalar(-0.293436), cbtScalar(-0.858252), cbtScalar(-0.420860)),
			cbtVector3(cbtScalar(-0.175767), cbtScalar(-0.780677), cbtScalar(-0.599262)),
			cbtVector3(cbtScalar(-0.170108), cbtScalar(-0.858835), cbtScalar(0.482865)),
			cbtVector3(cbtScalar(-0.332854), cbtScalar(-0.635055), cbtScalar(-0.696857)),
			cbtVector3(cbtScalar(-0.447791), cbtScalar(-0.445299), cbtScalar(-0.775128)),
			cbtVector3(cbtScalar(-0.470622), cbtScalar(-0.074146), cbtScalar(-0.879164)),
			cbtVector3(cbtScalar(-0.639417), cbtScalar(-0.340505), cbtScalar(-0.689049)),
			cbtVector3(cbtScalar(-0.598438), cbtScalar(0.104722), cbtScalar(-0.794256)),
			cbtVector3(cbtScalar(-0.488575), cbtScalar(0.307699), cbtScalar(-0.816313)),
			cbtVector3(cbtScalar(-0.379882), cbtScalar(0.513592), cbtScalar(-0.769077)),
			cbtVector3(cbtScalar(-0.425740), cbtScalar(0.862775), cbtScalar(-0.272516)),
			cbtVector3(cbtScalar(-0.480769), cbtScalar(0.875412), cbtScalar(-0.048439)),
			cbtVector3(cbtScalar(-0.467890), cbtScalar(0.648716), cbtScalar(-0.600043)),
			cbtVector3(cbtScalar(-0.543799), cbtScalar(0.730956), cbtScalar(-0.411881)),
			cbtVector3(cbtScalar(-0.516284), cbtScalar(0.838277), cbtScalar(0.174076)),
			cbtVector3(cbtScalar(-0.353343), cbtScalar(0.876384), cbtScalar(0.326519)),
			cbtVector3(cbtScalar(-0.572875), cbtScalar(0.614497), cbtScalar(0.542007)),
			cbtVector3(cbtScalar(-0.503600), cbtScalar(0.497261), cbtScalar(0.706161)),
			cbtVector3(cbtScalar(-0.530920), cbtScalar(0.754870), cbtScalar(0.384685)),
			cbtVector3(cbtScalar(-0.395884), cbtScalar(0.366414), cbtScalar(0.841818)),
			cbtVector3(cbtScalar(-0.300656), cbtScalar(0.001678), cbtScalar(0.953661)),
			cbtVector3(cbtScalar(-0.461060), cbtScalar(0.146912), cbtScalar(0.875000)),
			cbtVector3(cbtScalar(-0.315486), cbtScalar(-0.232212), cbtScalar(0.919893)),
			cbtVector3(cbtScalar(-0.323682), cbtScalar(-0.449187), cbtScalar(0.832644)),
			cbtVector3(cbtScalar(-0.318999), cbtScalar(-0.639527), cbtScalar(0.699134)),
			cbtVector3(cbtScalar(-0.496771), cbtScalar(-0.866029), cbtScalar(-0.055271)),
			cbtVector3(cbtScalar(-0.496771), cbtScalar(-0.816257), cbtScalar(-0.294377)),
			cbtVector3(cbtScalar(-0.456377), cbtScalar(-0.869528), cbtScalar(0.188130)),
			cbtVector3(cbtScalar(-0.380858), cbtScalar(-0.827144), cbtScalar(0.412792)),
			cbtVector3(cbtScalar(-0.449352), cbtScalar(-0.727405), cbtScalar(-0.518259)),
			cbtVector3(cbtScalar(-0.570533), cbtScalar(-0.551064), cbtScalar(-0.608632)),
			cbtVector3(cbtScalar(-0.656394), cbtScalar(-0.118280), cbtScalar(-0.744874)),
			cbtVector3(cbtScalar(-0.756696), cbtScalar(-0.438105), cbtScalar(-0.484882)),
			cbtVector3(cbtScalar(-0.801773), cbtScalar(-0.204798), cbtScalar(-0.561005)),
			cbtVector3(cbtScalar(-0.785186), cbtScalar(0.038618), cbtScalar(-0.617805)),
			cbtVector3(cbtScalar(-0.709082), cbtScalar(0.262399), cbtScalar(-0.654306)),
			cbtVector3(cbtScalar(-0.583412), cbtScalar(0.462265), cbtScalar(-0.667383)),
			cbtVector3(cbtScalar(-0.616001), cbtScalar(0.761286), cbtScalar(-0.201272)),
			cbtVector3(cbtScalar(-0.660687), cbtScalar(0.750204), cbtScalar(0.020072)),
			cbtVector3(cbtScalar(-0.744987), cbtScalar(0.435823), cbtScalar(-0.504791)),
			cbtVector3(cbtScalar(-0.713765), cbtScalar(0.605554), cbtScalar(-0.351373)),
			cbtVector3(cbtScalar(-0.686251), cbtScalar(0.687600), cbtScalar(0.236927)),
			cbtVector3(cbtScalar(-0.680201), cbtScalar(0.429407), cbtScalar(0.593732)),
			cbtVector3(cbtScalar(-0.733474), cbtScalar(0.546450), cbtScalar(0.403814)),
			cbtVector3(cbtScalar(-0.591023), cbtScalar(0.292923), cbtScalar(0.751445)),
			cbtVector3(cbtScalar(-0.500283), cbtScalar(-0.080757), cbtScalar(0.861922)),
			cbtVector3(cbtScalar(-0.643710), cbtScalar(0.070115), cbtScalar(0.761985)),
			cbtVector3(cbtScalar(-0.506332), cbtScalar(-0.308425), cbtScalar(0.805122)),
			cbtVector3(cbtScalar(-0.503015), cbtScalar(-0.509847), cbtScalar(0.697573)),
			cbtVector3(cbtScalar(-0.482525), cbtScalar(-0.682105), cbtScalar(0.549229)),
			cbtVector3(cbtScalar(-0.680396), cbtScalar(-0.716323), cbtScalar(-0.153451)),
			cbtVector3(cbtScalar(-0.658346), cbtScalar(-0.746264), cbtScalar(0.097562)),
			cbtVector3(cbtScalar(-0.653272), cbtScalar(-0.646915), cbtScalar(-0.392948)),
			cbtVector3(cbtScalar(-0.590828), cbtScalar(-0.732655), cbtScalar(0.337645)),
			cbtVector3(cbtScalar(-0.819140), cbtScalar(-0.518013), cbtScalar(-0.246166)),
			cbtVector3(cbtScalar(-0.900513), cbtScalar(-0.282178), cbtScalar(-0.330487)),
			cbtVector3(cbtScalar(-0.914953), cbtScalar(-0.028652), cbtScalar(-0.402122)),
			cbtVector3(cbtScalar(-0.859924), cbtScalar(0.220209), cbtScalar(-0.459898)),
			cbtVector3(cbtScalar(-0.777185), cbtScalar(0.613720), cbtScalar(-0.137836)),
			cbtVector3(cbtScalar(-0.805285), cbtScalar(0.586889), cbtScalar(0.082728)),
			cbtVector3(cbtScalar(-0.872413), cbtScalar(0.406077), cbtScalar(-0.271735)),
			cbtVector3(cbtScalar(-0.859339), cbtScalar(0.448072), cbtScalar(0.246101)),
			cbtVector3(cbtScalar(-0.757671), cbtScalar(0.216320), cbtScalar(0.615594)),
			cbtVector3(cbtScalar(-0.826165), cbtScalar(0.348139), cbtScalar(0.442851)),
			cbtVector3(cbtScalar(-0.671810), cbtScalar(-0.162803), cbtScalar(0.722557)),
			cbtVector3(cbtScalar(-0.796504), cbtScalar(-0.004543), cbtScalar(0.604468)),
			cbtVector3(cbtScalar(-0.676298), cbtScalar(-0.378223), cbtScalar(0.631794)),
			cbtVector3(cbtScalar(-0.668883), cbtScalar(-0.558258), cbtScalar(0.490673)),
			cbtVector3(cbtScalar(-0.821287), cbtScalar(-0.570118), cbtScalar(0.006994)),
			cbtVector3(cbtScalar(-0.767428), cbtScalar(-0.587810), cbtScalar(0.255470)),
			cbtVector3(cbtScalar(-0.933296), cbtScalar(-0.349837), cbtScalar(-0.079865)),
			cbtVector3(cbtScalar(-0.982667), cbtScalar(-0.100393), cbtScalar(-0.155208)),
			cbtVector3(cbtScalar(-0.961396), cbtScalar(0.160910), cbtScalar(-0.222938)),
			cbtVector3(cbtScalar(-0.934858), cbtScalar(0.354555), cbtScalar(-0.006864)),
			cbtVector3(cbtScalar(-0.941687), cbtScalar(0.229736), cbtScalar(0.245711)),
			cbtVector3(cbtScalar(-0.884317), cbtScalar(0.131552), cbtScalar(0.447536)),
			cbtVector3(cbtScalar(-0.810359), cbtScalar(-0.219769), cbtScalar(0.542788)),
			cbtVector3(cbtScalar(-0.915929), cbtScalar(-0.210048), cbtScalar(0.341743)),
			cbtVector3(cbtScalar(-0.816799), cbtScalar(-0.407192), cbtScalar(0.408303)),
			cbtVector3(cbtScalar(-0.903050), cbtScalar(-0.392416), cbtScalar(0.174076)),
			cbtVector3(cbtScalar(-0.980325), cbtScalar(-0.170969), cbtScalar(0.096586)),
			cbtVector3(cbtScalar(-0.995936), cbtScalar(0.084891), cbtScalar(0.029441)),
			cbtVector3(cbtScalar(-0.960031), cbtScalar(0.002650), cbtScalar(0.279283)),
		};
	static cbtVector3 sUnitSpherePoints[NUM_UNITSPHERE_POINTS + MAX_PREFERRED_PENETRATION_DIRECTIONS * 2] =
		{
			cbtVector3(cbtScalar(0.000000), cbtScalar(-0.000000), cbtScalar(-1.000000)),
			cbtVector3(cbtScalar(0.723608), cbtScalar(-0.525725), cbtScalar(-0.447219)),
			cbtVector3(cbtScalar(-0.276388), cbtScalar(-0.850649), cbtScalar(-0.447219)),
			cbtVector3(cbtScalar(-0.894426), cbtScalar(-0.000000), cbtScalar(-0.447216)),
			cbtVector3(cbtScalar(-0.276388), cbtScalar(0.850649), cbtScalar(-0.447220)),
			cbtVector3(cbtScalar(0.723608), cbtScalar(0.525725), cbtScalar(-0.447219)),
			cbtVector3(cbtScalar(0.276388), cbtScalar(-0.850649), cbtScalar(0.447220)),
			cbtVector3(cbtScalar(-0.723608), cbtScalar(-0.525725), cbtScalar(0.447219)),
			cbtVector3(cbtScalar(-0.723608), cbtScalar(0.525725), cbtScalar(0.447219)),
			cbtVector3(cbtScalar(0.276388), cbtScalar(0.850649), cbtScalar(0.447219)),
			cbtVector3(cbtScalar(0.894426), cbtScalar(0.000000), cbtScalar(0.447216)),
			cbtVector3(cbtScalar(-0.000000), cbtScalar(0.000000), cbtScalar(1.000000)),
			cbtVector3(cbtScalar(0.425323), cbtScalar(-0.309011), cbtScalar(-0.850654)),
			cbtVector3(cbtScalar(-0.162456), cbtScalar(-0.499995), cbtScalar(-0.850654)),
			cbtVector3(cbtScalar(0.262869), cbtScalar(-0.809012), cbtScalar(-0.525738)),
			cbtVector3(cbtScalar(0.425323), cbtScalar(0.309011), cbtScalar(-0.850654)),
			cbtVector3(cbtScalar(0.850648), cbtScalar(-0.000000), cbtScalar(-0.525736)),
			cbtVector3(cbtScalar(-0.525730), cbtScalar(-0.000000), cbtScalar(-0.850652)),
			cbtVector3(cbtScalar(-0.688190), cbtScalar(-0.499997), cbtScalar(-0.525736)),
			cbtVector3(cbtScalar(-0.162456), cbtScalar(0.499995), cbtScalar(-0.850654)),
			cbtVector3(cbtScalar(-0.688190), cbtScalar(0.499997), cbtScalar(-0.525736)),
			cbtVector3(cbtScalar(0.262869), cbtScalar(0.809012), cbtScalar(-0.525738)),
			cbtVector3(cbtScalar(0.951058), cbtScalar(0.309013), cbtScalar(0.000000)),
			cbtVector3(cbtScalar(0.951058), cbtScalar(-0.309013), cbtScalar(0.000000)),
			cbtVector3(cbtScalar(0.587786), cbtScalar(-0.809017), cbtScalar(0.000000)),
			cbtVector3(cbtScalar(0.000000), cbtScalar(-1.000000), cbtScalar(0.000000)),
			cbtVector3(cbtScalar(-0.587786), cbtScalar(-0.809017), cbtScalar(0.000000)),
			cbtVector3(cbtScalar(-0.951058), cbtScalar(-0.309013), cbtScalar(-0.000000)),
			cbtVector3(cbtScalar(-0.951058), cbtScalar(0.309013), cbtScalar(-0.000000)),
			cbtVector3(cbtScalar(-0.587786), cbtScalar(0.809017), cbtScalar(-0.000000)),
			cbtVector3(cbtScalar(-0.000000), cbtScalar(1.000000), cbtScalar(-0.000000)),
			cbtVector3(cbtScalar(0.587786), cbtScalar(0.809017), cbtScalar(-0.000000)),
			cbtVector3(cbtScalar(0.688190), cbtScalar(-0.499997), cbtScalar(0.525736)),
			cbtVector3(cbtScalar(-0.262869), cbtScalar(-0.809012), cbtScalar(0.525738)),
			cbtVector3(cbtScalar(-0.850648), cbtScalar(0.000000), cbtScalar(0.525736)),
			cbtVector3(cbtScalar(-0.262869), cbtScalar(0.809012), cbtScalar(0.525738)),
			cbtVector3(cbtScalar(0.688190), cbtScalar(0.499997), cbtScalar(0.525736)),
			cbtVector3(cbtScalar(0.525730), cbtScalar(0.000000), cbtScalar(0.850652)),
			cbtVector3(cbtScalar(0.162456), cbtScalar(-0.499995), cbtScalar(0.850654)),
			cbtVector3(cbtScalar(-0.425323), cbtScalar(-0.309011), cbtScalar(0.850654)),
			cbtVector3(cbtScalar(-0.425323), cbtScalar(0.309011), cbtScalar(0.850654)),
			cbtVector3(cbtScalar(0.162456), cbtScalar(0.499995), cbtScalar(0.850654))};
	if (highres)
		return sUnitSpherePointsHighres;
	return sUnitSpherePoints;
}
