using UnityEngine;
using System.Collections;
using System;
using AnnexR.Primitives;

namespace AnnexR{
	public class AnnexRComponent : MonoBehaviour {

		public Shape primitiveType;
		public float primitivePref;
		public BaseParams primitiveParams;

		// Private vars for rendering purposes
		[SerializeField] private bool showWireframe;
		[SerializeField] private bool showHandles;
		[SerializeField] private bool showPositionHandle;
		[SerializeField] private bool showRotationHandle;
		[SerializeField] private bool showMinMaxHandle;
		[SerializeField] private Color handleColor;
		[SerializeField] private Color wireframeColor;

		// Temporarily hold primitive specific params. Later assigned to BaseParams
		[SerializeField] private PlaneParams planeParams;
		[SerializeField] private CylinderParams cylinderParams;
		[SerializeField] private ConeParams coneParams;
		[SerializeField] private SphereParams sphereParams;
		[SerializeField] private TorusParams torusParams;

		public AnnexRComponent(){
			//Debug.Log ("Contruct Comp");
			Debug.Log (gameObject.transform.root.name + " " +  gameObject.transform.root.childCount.ToString());
			Transform rootTrackingTrans = gameObject.transform.root.GetChild(3);
			Debug.Log (rootTrackingTrans.name);

			primitivePref = 1;
			primitiveType = Shape.None;
			primitiveParams = (NoneParams)ScriptableObject.CreateInstance(typeof(NoneParams));

			showWireframe = true;
			showHandles = true;
			showPositionHandle = false;
			showRotationHandle = false;
			showMinMaxHandle = false;
			handleColor = Color.red;
			wireframeColor = Color.green;

			planeParams = (PlaneParams)ScriptableObject.CreateInstance(typeof(PlaneParams));
			planeParams.center = rootTrackingTrans.transform.InverseTransformPoint(
				gameObject.GetComponent<Renderer>().bounds.center);
			planeParams.normal = rootTrackingTrans.transform.InverseTransformDirection(
				-gameObject.transform.forward);
			planeParams.localRight = rootTrackingTrans.transform.InverseTransformDirection(
				gameObject.transform.right);
			planeParams.localUp = rootTrackingTrans.transform.InverseTransformDirection(
				gameObject.transform.up);
			planeParams.width = 2f * gameObject.GetComponent<MeshFilter> ().sharedMesh.bounds.extents.x * gameObject.transform.localScale.x;
			planeParams.length = 2f * gameObject.GetComponent<MeshFilter> ().sharedMesh.bounds.extents.y * gameObject.transform.localScale.y;
			planeParams.widthMin = planeParams.width * 0.5f;
			planeParams.widthMax = planeParams.width * 1.5f;
			planeParams.lengthMin = planeParams.length * 0.5f;
			planeParams.lengthMax = planeParams.length * 1.5f;

			sphereParams = (SphereParams)ScriptableObject.CreateInstance(typeof(SphereParams));
			sphereParams.center = rootTrackingTrans.transform.InverseTransformPoint(
				gameObject.GetComponent<Renderer>().bounds.center);
			sphereParams.localRight = rootTrackingTrans.transform.InverseTransformDirection(
				gameObject.transform.right);
			sphereParams.localUp = rootTrackingTrans.transform.InverseTransformDirection(
				gameObject.transform.up);
			sphereParams.localForward = rootTrackingTrans.transform.InverseTransformDirection(
				gameObject.transform.forward);
			sphereParams.radius = gameObject.GetComponent<MeshFilter> ().sharedMesh.bounds.extents.x * gameObject.transform.localScale.x;
			sphereParams.radiusMin = sphereParams.radius * 0.5f;
			sphereParams.radiusMax = sphereParams.radius * 1.5f;

			cylinderParams = (CylinderParams)ScriptableObject.CreateInstance(typeof(CylinderParams));
			cylinderParams.axisPosition = rootTrackingTrans.transform.InverseTransformPoint(
				gameObject.GetComponent<Renderer>().bounds.center);
			cylinderParams.axisDirection = rootTrackingTrans.transform.InverseTransformDirection(
				gameObject.transform.up);
			cylinderParams.localRight = rootTrackingTrans.transform.InverseTransformDirection(
				gameObject.transform.right);
			cylinderParams.localForward = rootTrackingTrans.transform.InverseTransformDirection(
				gameObject.transform.forward);
			cylinderParams.radius = gameObject.GetComponent<MeshFilter> ().sharedMesh.bounds.extents.x * gameObject.transform.localScale.x;
			cylinderParams.height = 2f * gameObject.GetComponent<MeshFilter> ().sharedMesh.bounds.extents.y * gameObject.transform.localScale.y;
			cylinderParams.radiusMin = cylinderParams.radius * 0.5f;
			cylinderParams.radiusMax = cylinderParams.radius * 1.5f;
			cylinderParams.heightMin = cylinderParams.height * 0.5f;
			cylinderParams.heightMax = cylinderParams.height * 1.5f;

			coneParams = (ConeParams)ScriptableObject.CreateInstance(typeof(ConeParams));
			coneParams.center = rootTrackingTrans.transform.InverseTransformPoint(
				gameObject.GetComponent<Renderer>().bounds.center);
			coneParams.axisDirection = rootTrackingTrans.transform.InverseTransformDirection(
				gameObject.transform.up);
			coneParams.localRight = rootTrackingTrans.transform.InverseTransformDirection(
				gameObject.transform.right);
			coneParams.localForward = rootTrackingTrans.transform.InverseTransformDirection(
				gameObject.transform.forward);
			coneParams.height = 2f * gameObject.GetComponent<MeshFilter> ().sharedMesh.bounds.extents.y * gameObject.transform.localScale.y;
			float rad = gameObject.GetComponent<MeshFilter> ().sharedMesh.bounds.extents.x * gameObject.transform.localScale.x;
			if (rad > 0)
				coneParams.angle = Mathf.Atan(rad / (coneParams.height != 0 ? coneParams.height : rad))* 180f / Mathf.PI;
			else
				coneParams.angle = 45f;
			coneParams.heightMin = coneParams.height * 0.5f;
			coneParams.heightMax = coneParams.height * 1.5f;
			coneParams.angleMin = coneParams.angle * 0.5f;
			coneParams.angleMax = coneParams.angle * 1.1f;

			torusParams = (TorusParams)ScriptableObject.CreateInstance(typeof(TorusParams));
			torusParams.center = rootTrackingTrans.transform.InverseTransformPoint(
				gameObject.GetComponent<Renderer>().bounds.center);
			torusParams.axisDirection = rootTrackingTrans.transform.InverseTransformDirection(
				gameObject.transform.up);
			torusParams.localRight = rootTrackingTrans.transform.InverseTransformDirection(
				gameObject.transform.right);
			torusParams.localForward = rootTrackingTrans.transform.InverseTransformDirection(
				gameObject.transform.forward);
			torusParams.maxRadius = gameObject.GetComponent<MeshFilter> ().sharedMesh.bounds.extents.x * gameObject.transform.localScale.x * 1.5f;
			torusParams.minRadius = gameObject.GetComponent<MeshFilter> ().sharedMesh.bounds.extents.x * gameObject.transform.localScale.x * 0.5f;
			torusParams.minRadiusMin = torusParams.minRadius * 0.5f;
			torusParams.minRadiusMax = torusParams.minRadius * 1.5f;
			torusParams.maxRadiusMin = torusParams.maxRadius * 0.5f;
			torusParams.maxRadiusMax = torusParams.maxRadius * 1.5f;
		}

		void Awake(){
			//Debug.Log ("Awake Comp");
		}

		void OnEnable(){
			//Debug.Log ("Enable Comp");
		}
		
		void Start(){
			//Debug.Log ("Start Comp");
		}

		void OnDestroy(){
			//Debug.Log ("Destroy Comp");
			//gameObject.tag = "Untagged";
		}

		public void mprint(string str){
			print ("----------------------------------------------------------------------------- from "+ str +" ---------------------------------------------------------------------------------\n");
			print ("primitivePref " + primitivePref.ToString() + "\n");
			try{
				PlaneParams param = (PlaneParams)primitiveParams;
				print ("Type: " + param.ToString() + ",  " +
				       "Width: " + param.width + ",  " +
				       "Length: " + param.length + ",  " +
				       "WidthPref: " + param.widthPref + ",  " +
				       "LengthPref: " + param.lengthPref + "\n");
			}catch(InvalidCastException e){}
			try{
				SphereParams param = (SphereParams)primitiveParams;
				print ("Type: " + param.ToString() + ",  " +
				       "Radius: " + param.radius + ",  " + 
				       "RadiusPref: " + param.radiusPref + "\n");
			}catch(InvalidCastException e){}
			try{
				CylinderParams param = (CylinderParams)primitiveParams;
				print ("Type: " + param.ToString() + "\n");
			}catch(InvalidCastException e){}
			try{
				ConeParams param = (ConeParams)primitiveParams;
				print ("Type: " + param.ToString() + "\n");
			}catch(InvalidCastException e){}
			try{
				TorusParams param = (TorusParams)primitiveParams;
				print ("Type: " + param.ToString() + "\n");
			}catch(InvalidCastException e){}
			print ("-\n");
		}
	}
}