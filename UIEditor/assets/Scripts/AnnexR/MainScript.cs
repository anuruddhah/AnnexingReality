using UnityEngine;
using System.Collections;
using AnnexR;
using AnnexR.Primitives;
using System.Collections.Generic;

public class MainScript : MonoBehaviour {

	GameObject[] arObjects;
	AnnexRComponent[][] arComponents;

	JSONObject jObj = new JSONObject (JSONObject.Type.OBJECT);
	JSONObject jShapes = new JSONObject (JSONObject.Type.ARRAY);
	string jStr = "{}";

	Vector3[] arObjInitPosition;
	Quaternion[] arObjInitRotation;
	Vector3[] arObjInitScale;

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
		arComponents = new AnnexRComponent[arObjects.Length][];
		Debug.Log ("Number of objects " + arComponents.Length.ToString()); // Debug

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

			arComponents[id] =  arObjects[id].GetComponentsInChildren<AnnexRComponent>();
			Debug.Log ("---------- Number of components in object " + id + ": " + arComponents[id].Length.ToString()); // Debug

			foreach(AnnexRComponent arComp in arComponents[id]){
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

				Debug.Log (arComp.name);
			}
		}

		jStr = jObj.Print ();
		Debug.Log (jStr);

	}
	
	void Update () {
		for (int ObjId = 0; ObjId < arObjects.Length; ObjId++) {
			
			Vector3 initPosition, initAxisDir;
			Quaternion initRot;
			
			switch(arComponents[ObjId][0].primitiveType){
			case Shape.Plane:
				PlaneParams planeParams = arComponents[ObjId][0].primitiveParams as PlaneParams;
				initPosition = planeParams.center;
				initAxisDir = planeParams.normal;
				initRot = Quaternion.FromToRotation(Vector3.back, initAxisDir);
				break;
			case Shape.Sphere:
				SphereParams sphereParams = arComponents[ObjId][0].primitiveParams as SphereParams;
				initPosition = sphereParams.center;
				initAxisDir = sphereParams.localUp;
				initRot = Quaternion.FromToRotation(Vector3.up, initAxisDir);
				break;
			case Shape.Cylinder:
				CylinderParams cylinerParams = arComponents[ObjId][0].primitiveParams as CylinderParams;
				initPosition = cylinerParams.axisPosition;
				initAxisDir = cylinerParams.axisDirection;
				initRot = Quaternion.FromToRotation(Vector3.up, initAxisDir);
				break;
			case Shape.Cone:
				ConeParams coneParams = arComponents[ObjId][0].primitiveParams as ConeParams;
				initPosition = coneParams.center;
				initAxisDir = coneParams.axisDirection;
				initRot = Quaternion.FromToRotation(Vector3.up, initAxisDir);
				break;
			case Shape.Torus:
				TorusParams torusParams = arComponents[ObjId][0].primitiveParams as TorusParams;
				initPosition = torusParams.center;
				initAxisDir = torusParams.axisDirection;
				initRot = Quaternion.FromToRotation(Vector3.up, initAxisDir);
				break;
			default:
				initPosition = new Vector3(0f, 0f, 0f);
				initAxisDir = new Vector3(0f, 0f, 0f);
				initRot = Quaternion.identity;
				break;
			}
			
			//arObjects[ObjId].transform.localPosition = translation;
			//arObjects[ObjId].transform.localRotation = qRotation;
			arObjects[ObjId].transform.parent = GameObject.FindGameObjectWithTag("RootTracking").transform;
			arObjects[ObjId].transform.localScale = arObjInitScale[ObjId];
			arObjects[ObjId].transform.localRotation = arObjInitRotation[ObjId];
			arObjects[ObjId].transform.localPosition = arObjInitPosition[ObjId];
			
			//GameObject parent = GameObject.CreatePrimitive(PrimitiveType.Cube);
			//parent.name = "Parent";
			GameObject.Destroy(GameObject.Find("Parent"));
			GameObject parent = new GameObject("Parent");
			parent.transform.parent = GameObject.FindGameObjectWithTag("RootTracking").transform;
			parent.transform.localScale = new Vector3(1f, 1f, 1f);
			parent.transform.localRotation = initRot;
			parent.transform.localPosition = initPosition;
			
			//			GameObject ZeroScaleparent = new GameObject("ZeroScaleParent");
			//			ZeroScaleparent.transform.parent = parent.transform;
			//			ZeroScaleparent.transform.localPosition = new Vector3(0f, 0f, 0f);
			//			ZeroScaleparent.transform.localRotation = Quaternion.identity;
			
			//Vector3 localPosition = arObjects[ObjId].transform.localPosition;
			
			arObjects[ObjId].transform.parent = parent.transform;
			
			Vector3 localScale = arObjects[ObjId].transform.localScale;
			Quaternion localRotation = arObjects[ObjId].transform.localRotation;
			Vector3 localPosition = arObjects[ObjId].transform.localPosition;
			
			parent.transform.localScale = new Vector3(1f, 5f, 1f);
			parent.transform.localRotation = Quaternion.identity;//qRotation;
			parent.transform.localPosition = new Vector3(0f, 0f, 0f);//currentPosition;
			
			Debug.Log(arObjects[ObjId].transform.position);
			Debug.Log(arObjects[ObjId].transform.rotation);
			Debug.Log(arObjects[ObjId].transform.lossyScale);
			
			//parent.transform.DetachChildren();
			//arObjects[ObjId].transform.parent = GameObject.FindGameObjectWithTag("RootTracking").transform;
			
			//GameObject.Destroy(ZeroScaleparent);
			//GameObject.Destroy(parent);
			Debug.Log("END");
		}
	}

	void OnDestroy(){
		Debug.Log("DESTROY");
	}
}
