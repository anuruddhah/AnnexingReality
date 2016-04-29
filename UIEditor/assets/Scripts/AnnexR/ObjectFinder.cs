using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices;
using AnnexR;
using AnnexR.Primitives;
using System.Collections.Generic;

public class ObjectFinder : MonoBehaviour {

	[DllImport ("ObjectFinder")]
	private static extern int setShapes (string shapes);
	
	[DllImport ("ObjectFinder")]
	private static extern void start ();
	
	[DllImport ("ObjectFinder")]
	private static extern int getPoses (int shapeCount, [In, Out] int[] types, [In, Out] float[] dims, [In, Out] float[] trans, [In, Out] float[] rot);

	[DllImport ("ObjectFinder")]
	private static extern void restart ();

	[DllImport ("ObjectFinder")]
	private static extern void stop ();

	GameObject[] arObjects;
	Vector3[] arObjInitPosition;
	Quaternion[] arObjInitRotation;
	Vector3[] arObjInitScale;
	Dictionary<Shape, AnnexRComponent>[] arComponents;
	
	JSONObject jObj = new JSONObject (JSONObject.Type.OBJECT);
	JSONObject jShapes = new JSONObject (JSONObject.Type.ARRAY);
	string jStr = "{}";

	int objectFinderCode = -1;
	
	const string ARR_SHAPES = "Shapes";
	const string ARR_PRIMITIVES = "Primitives";
	const string PRIMITIVE_TYPE = "PrimShape";
	const string PRIMITIVE_PARAMS = "BaseTypeParams";
	
	const string TYPE_PLANE = "PLANE";
	const string TYPE_CYLINDER = "CYLINDER";
	const string TYPE_CONE = "CONE";
	const string TYPE_SPHERE = "SPHERE";
	const string TYPE_TORUS = "TORUS";
	const string TYPE_NONE = "NONE";
	
	const string OBJ_PREFERENCE = "objectPref";
	const string OBJ_ALLOW_JUMP = "jumpAllowed";
	
	const string PRIM_PREFERENCE = "primitivePref";
	
	const string PLANE_WIDTH = "width";
	const string PLANE_LENGHT = "length";
	const string PLANE_WIDTH_PREF = "widthPref";
	const string PLANE_LENGHT_PREF = "lengthPref";
	const string PLANE_RATIO_PREF = "ratioPref";
	
	const string CYLINDER_RADIUS = "radius";
	const string CYLINDER_HEIGHT = "height";
	const string CYLINDER_RADIUS_PREF = "radiusPref";
	const string CYLINDER_HEIGHT_PREF = "heightPref";
	const string CYLINDER_RATIO_PREF = "ratioPref";
	
	const string CONE_ANGLE = "angle";
	const string CONE_HEIGHT = "height";
	const string CONE_ANGLE_PREF = "anglePref";
	const string CONE_HEIGHT_PREF = "heightPref";
	const string CONE_RATIO_PREF = "ratioPref";
	
	const string SPHERE_RADIUS = "radius";
	const string SPHERE_RADIUS_PREF = "radiusPref";
	
	const string TORUS_MIN_RADIUS = "minRadius";
	const string TORUS_MAX_RADIUS = "maxRadius";
	const string TORUS_MIN_RADIUS_PREF = "minRadiusPref";
	const string TORUS_MAX_RADIUS_PREF = "maxRadiusPref";
	const string TORUS_RATIO_PREF = "ratioPref";

	void Start () {
		arObjects = GameObject.FindGameObjectsWithTag ("AnnexingRealityObject");
		arObjInitPosition = new Vector3[arObjects.Length];
		arObjInitRotation = new Quaternion[arObjects.Length];
		arObjInitScale = new Vector3[arObjects.Length];
		arComponents = new Dictionary<Shape, AnnexRComponent>[arObjects.Length];
		Debug.Log ("Number of objects " + arObjects.Length.ToString()); // Debug
		
		jObj.AddField (ARR_SHAPES, jShapes);
		
		for(int id = 0; id < arObjects.Length; id++){
			JSONObject jShape = new JSONObject(JSONObject.Type.OBJECT);
			jShapes.Add(jShape);
			jShape.AddField(OBJ_PREFERENCE, arObjects[id].GetComponent<AnnexRObject>().objectPref);
			jShape.AddField(OBJ_ALLOW_JUMP, arObjects[id].GetComponent<AnnexRObject>().allowJumping);
			
			JSONObject jPrimitives = new JSONObject (JSONObject.Type.ARRAY);
			jShape.AddField(ARR_PRIMITIVES, jPrimitives);

			arObjInitPosition[id] = arObjects[id].transform.localPosition;
			arObjInitRotation[id] = arObjects[id].transform.localRotation;
			arObjInitScale[id] = arObjects[id].transform.localScale;
			AnnexRComponent[] componentArr = arObjects[id].GetComponentsInChildren<AnnexRComponent>();
			SortComponents(ref componentArr);
			Dictionary<Shape, AnnexRComponent> componentDict = new Dictionary<Shape, AnnexRComponent>();
			Debug.Log ("Number of components in object " + id + ": " + componentArr.Length.ToString()); // Debug
			
			foreach(AnnexRComponent arComp in componentArr){
				Debug.Log (arComp.name); // Debug
				componentDict.Add(arComp.primitiveType, arComp);

				JSONObject jPrimitive = new JSONObject(JSONObject.Type.OBJECT);
				jPrimitives.Add(jPrimitive);
				JSONObject jParams = new JSONObject(JSONObject.Type.OBJECT);
				jParams.AddField(PRIM_PREFERENCE, arComp.primitivePref);
				
				switch(arComp.primitiveType){
				case Shape.Plane:
					jPrimitive.AddField(PRIMITIVE_TYPE, TYPE_PLANE);
					jPrimitive.AddField(PRIMITIVE_PARAMS, jParams);
					PlaneParams planeParams = arComp.primitiveParams as PlaneParams;
					jParams.AddField(PLANE_WIDTH, planeParams.width);
					jParams.AddField(PLANE_LENGHT, planeParams.length);
					jParams.AddField(PLANE_WIDTH_PREF, planeParams.widthPref);
					jParams.AddField(PLANE_LENGHT_PREF, planeParams.lengthPref);
					jParams.AddField(PLANE_RATIO_PREF, planeParams.ratioPref);
					break;
				case Shape.Sphere:
					jPrimitive.AddField(PRIMITIVE_TYPE, TYPE_SPHERE);
					jPrimitive.AddField(PRIMITIVE_PARAMS, jParams);
					SphereParams sphereParams = arComp.primitiveParams as SphereParams;
					jParams.AddField(SPHERE_RADIUS, sphereParams.radius);
					jParams.AddField(SPHERE_RADIUS_PREF, sphereParams.radiusPref);
					break;
				case Shape.Cylinder:
					jPrimitive.AddField(PRIMITIVE_TYPE, TYPE_CYLINDER);
					jPrimitive.AddField(PRIMITIVE_PARAMS, jParams);
					CylinderParams cylinerParams = arComp.primitiveParams as CylinderParams;
					jParams.AddField(CYLINDER_RADIUS, cylinerParams.radius);
					jParams.AddField(CYLINDER_HEIGHT, cylinerParams.height);
					jParams.AddField(CYLINDER_RADIUS_PREF, cylinerParams.radiusPref);
					jParams.AddField(CYLINDER_HEIGHT_PREF, cylinerParams.heightPref);
					jParams.AddField(CYLINDER_RATIO_PREF, cylinerParams.ratioPref);
					break;
				case Shape.Cone:
					jPrimitive.AddField(PRIMITIVE_TYPE, TYPE_CONE);
					jPrimitive.AddField(PRIMITIVE_PARAMS, jParams);
					ConeParams coneParams = arComp.primitiveParams as ConeParams;
					jParams.AddField(CONE_ANGLE, coneParams.angle);
					jParams.AddField(CONE_HEIGHT, coneParams.height);
					jParams.AddField(CONE_ANGLE_PREF, coneParams.anglePref);
					jParams.AddField(CONE_HEIGHT_PREF, coneParams.heightPref);
					jParams.AddField(CONE_RATIO_PREF, coneParams.ratioPref);
					break;
				case Shape.Torus:
					jPrimitive.AddField(PRIMITIVE_TYPE, TYPE_TORUS);
					jPrimitive.AddField(PRIMITIVE_PARAMS, jParams);
					TorusParams torusParams = arComp.primitiveParams as TorusParams;
					jParams.AddField(TORUS_MIN_RADIUS, torusParams.minRadius);
					jParams.AddField(TORUS_MAX_RADIUS, torusParams.maxRadius);
					jParams.AddField(TORUS_MIN_RADIUS_PREF, torusParams.minRadiusPref);
					jParams.AddField(TORUS_MAX_RADIUS_PREF, torusParams.maxRadiusPref);
					jParams.AddField(TORUS_RATIO_PREF, torusParams.ratioPref);
					break;
				default:
					jPrimitive.AddField(PRIMITIVE_TYPE, TYPE_NONE);
					jPrimitive.AddField(PRIMITIVE_PARAMS, jParams);
					break;
				}
			}

			arComponents[id] = componentDict;
		}

		jStr = jObj.Print ();
		Debug.Log (jStr); // Debug

		Screen.SetResolution(960, 540, true);
		Camera[] cameras = Camera.allCameras;
		foreach (Camera cam in cameras) {
			cam.aspect = 16.0f/9.0f;
		}

		Debug.Log ("SETTING SHAPES..");
		objectFinderCode = setShapes (jStr);
		if (objectFinderCode == 0) {
			Debug.Log ("STARTING DETECTION..");
			start ();
		}
	}
	
	void Update () {
		int shapeCountVals = arObjects.Length;
		int[] typesVals = new int[ shapeCountVals ];
		float[] dimsVals = new float[ 2 * shapeCountVals ];
		float[] translationVals = new float[ 3 * shapeCountVals ];
		float[] rotationVals = new float[ 4 * shapeCountVals ];
		for(int i = 0; i < typesVals.Length; i++){
			typesVals[i] = -1;
		}
		for(int i = 0; i < dimsVals.Length; i++){
			dimsVals[i] = 0.0f;
		}
		for (int i = 0; i < translationVals.Length; i++) {
			translationVals [i] = 0.0f;
		}
		for (int i = 0; i < rotationVals.Length; i++) {
			rotationVals [i] = 0.0f;
		}

		//Debug.Log ("GETTING POSES..");
		getPoses (shapeCountVals, typesVals, dimsVals, translationVals, rotationVals);

		for (int ObjId = 0; ObjId < shapeCountVals; ObjId++) {
			if(typesVals[ObjId] == -1)
				continue; // Or do something else in the game. e.g. put in a defalt pose

			Quaternion qRotationRH = 
				new Quaternion (rotationVals [4 * ObjId + 0], rotationVals [4 * ObjId + 1], rotationVals [4 * ObjId + 2], rotationVals [4 * ObjId + 3]);
			Vector3 euRotationRH = qRotationRH.eulerAngles;
			Vector3 euRotation = new Vector3(euRotationRH.x, -euRotationRH.y, -euRotationRH.z);

			// Rotation, translation, dimensions, and type of detected primitive for this object
			Quaternion qRotation = Quaternion.Euler(euRotation);
			Vector3 translation = new Vector3 (-translationVals [3 * ObjId + 0], translationVals [3 * ObjId + 1], translationVals [3 * ObjId + 2]);
			Vector2 dims = new Vector2(dimsVals[2 * ObjId + 0], dimsVals[2* ObjId + 1]);
			Shape type = (Shape)typesVals[ObjId];

			Debug.Log("Type: " + type.ToString());

			AnnexRComponent detectedComponent = arComponents[ObjId][type];
			Vector3 initPosition, initAxisDir, currentPosition, currentAxisDir;
			Quaternion initRot, currentRot;
			Vector3 scale = new Vector3(1f, 1f, 1f);

			switch(type){
			case Shape.Plane:
				PlaneParams planeParams = detectedComponent.primitiveParams as PlaneParams;
				initPosition = planeParams.center;
				initAxisDir = planeParams.normal;
				currentPosition = translation;
				currentAxisDir = qRotation * Vector3.back;
				initRot = Quaternion.FromToRotation(Vector3.back, initAxisDir);
				initRot = Quaternion.FromToRotation(initRot * Vector3.up, planeParams.localUp) * initRot;
				scale.x = Mathf.Min( Mathf.Max(planeParams.widthMin, dims.x), planeParams.widthMax) / planeParams.width;
				scale.y = Mathf.Min( Mathf.Max(planeParams.lengthMin, dims.y), planeParams.lengthMax)/planeParams.length;
				scale.z = 1f;
				break;
			case Shape.Sphere:
				SphereParams sphereParams = detectedComponent.primitiveParams as SphereParams;
				initPosition = sphereParams.center;
				initAxisDir = sphereParams.localUp;
				currentPosition = translation;
				currentAxisDir = qRotation * Vector3.up;
				initRot = Quaternion.FromToRotation(Vector3.up, initAxisDir);
				scale.x = scale.y = scale.z = 
					Mathf.Min( Mathf.Max(sphereParams.radiusMin, dims.x), sphereParams.radiusMax) / sphereParams.radius;
				break;
			case Shape.Cylinder:
				CylinderParams cylinerParams = detectedComponent.primitiveParams as CylinderParams;
				initPosition = cylinerParams.axisPosition;
				initAxisDir = cylinerParams.axisDirection;
				currentPosition = translation;
				currentAxisDir = qRotation * Vector3.up;
				initRot = Quaternion.FromToRotation(Vector3.up, initAxisDir);
				scale.x = scale.z = 
					Mathf.Min( Mathf.Max(cylinerParams.radiusMin, dims.x), cylinerParams.radiusMax) / cylinerParams.radius;
				scale.y = Mathf.Min( Mathf.Max(cylinerParams.heightMin, dims.y), cylinerParams.heightMax) / cylinerParams.height;
				break;
			case Shape.Cone:
				ConeParams coneParams = detectedComponent.primitiveParams as ConeParams;
				initPosition = coneParams.center;
				initAxisDir = coneParams.axisDirection;
				currentPosition = translation;
				currentAxisDir = qRotation * Vector3.up;
				initRot = Quaternion.FromToRotation(Vector3.up, initAxisDir);
				scale.x = scale.z = 
					Mathf.Tan(Mathf.Min( Mathf.Max(coneParams.angleMin, dims.x), coneParams.angleMax) * Mathf.PI / 180f) / 
						Mathf.Tan(coneParams.angle * Mathf.PI / 180f);
				scale.y = Mathf.Min( Mathf.Max(coneParams.heightMin, dims.y), coneParams.heightMax) / coneParams.height;
				break;
			case Shape.Torus:
				TorusParams torusParams = detectedComponent.primitiveParams as TorusParams;
				initPosition = torusParams.center;
				initAxisDir = torusParams.axisDirection;
				currentPosition = translation;
				currentAxisDir = qRotation * Vector3.up;
				initRot = Quaternion.FromToRotation(Vector3.up, initAxisDir);
				scale.x = scale.y = scale.z = 
					Mathf.Min( Mathf.Max(torusParams.maxRadiusMin, dims.y), torusParams.maxRadiusMax) / torusParams.maxRadius;
				break;
			default:
				initPosition = new Vector3(0f, 0f, 0f);
				initAxisDir = new Vector3(0f, 0f, 0f);
				currentPosition = new Vector3(0f, 0f, 0f);
				currentAxisDir = new Vector3(0f, 0f, 0f);
				initRot = Quaternion.identity;
				break;
			}

			GameObject.Destroy(GameObject.Find("Pose" + ObjId.ToString()));
			GameObject pose = GameObject.CreatePrimitive(PrimitiveType.Sphere);
			pose.name = "Pose" + ObjId.ToString();
			pose.transform.parent = GameObject.FindGameObjectWithTag("RootTracking").transform;
			pose.transform.localPosition = currentPosition;
			pose.transform.localRotation = Quaternion.identity;
			pose.transform.localScale = new Vector3(0.01f, 0.01f, 0.01f);

			arObjects[ObjId].transform.parent = GameObject.FindGameObjectWithTag("RootTracking").transform;
			arObjects[ObjId].transform.localScale = arObjInitScale[ObjId];
			arObjects[ObjId].transform.localRotation = arObjInitRotation[ObjId];
			arObjects[ObjId].transform.localPosition = arObjInitPosition[ObjId];

			GameObject.Destroy(GameObject.Find("Parent" + ObjId.ToString()));
			GameObject parent = new GameObject("Parent" + ObjId.ToString());
			//GameObject parent = GameObject.CreatePrimitive(PrimitiveType.Cube);
			//parent.name = "Parent" + ObjId.ToString();
			parent.transform.parent = GameObject.FindGameObjectWithTag("RootTracking").transform;
			parent.transform.localPosition = initPosition;
			parent.transform.localRotation = initRot;
			//parent.transform.localScale = new Vector3(0.02f, 0.02f, 0.02f);


			arObjects[ObjId].transform.parent = parent.transform;

			parent.transform.localScale = Vector3.Scale(scale, new Vector3(1.2f, 1.2f, 1.2f));
			parent.transform.localRotation = qRotation;
			parent.transform.localPosition = currentPosition;

			//print (arObjects[ObjId].name + " type   " + "(" + typesVals[ObjId].ToString ("#.##") + ")");
			print (arObjects[ObjId].name + " dims   " + "(" + dimsVals[2 * ObjId + 0].ToString ("#.###") + ", " + dimsVals[2* ObjId + 1].ToString ("#.###") + ")");
			print (arObjects[ObjId].name + " translation" + "(" + translationVals [3 * ObjId + 0].ToString ("#.###") + ", " + translationVals [3 * ObjId + 1].ToString ("#.###") + ", " + translationVals [3 * ObjId + 2].ToString ("#.###") + ")");
			print (arObjects[ObjId].name + " rotation   " + "(" + euRotation.x.ToString ("#.###") + ", " + euRotation.y.ToString ("#.###") + ", " + euRotation.z.ToString ("#.###") + ")");
		
			if(Input.GetKeyDown(KeyCode.Space) || Input.GetMouseButtonDown(0))
				restart();
		}
	}

	void OnDestroy(){
		Debug.Log ("STOPPING DETECTION..");
		stop ();
	}

	void SortComponents( ref AnnexRComponent[] componentArr){
		int minIndex;
		AnnexRComponent minPrefComponent;
		for (int targetIndex = (componentArr.Length - 1); targetIndex > 0; targetIndex--) {
			minIndex = 0;
			for(int currentIndex = 0; currentIndex <= targetIndex; currentIndex++){
				if(componentArr[minIndex].primitivePref > componentArr[currentIndex].primitivePref)
					minIndex = currentIndex;
			}
			minPrefComponent = componentArr[minIndex];
			componentArr[minIndex] = componentArr[targetIndex];
			componentArr[targetIndex] = minPrefComponent;
		}
	}

}
