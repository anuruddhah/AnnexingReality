using UnityEngine;
using UnityEditor;
using System.Collections;
using AnnexR.Primitives;
using AnnexR;

[CustomEditor(typeof(AnnexRComponent))]
public class AnnexRComponentEditor : Editor {
	// Object component instance this editor belongs to. Read only
	AnnexRComponent component;

	// Publicly exposed properties
	SerializedProperty primitiveTypeProp;
	SerializedProperty primitiveParamsProp;
	SerializedProperty primitivePrefProp;

	// private properties
	SerializedProperty showWireframeProp;
	SerializedProperty showHandlesProp;
	SerializedProperty showPositionHandleProp;
	SerializedProperty showRotationHandleProp;
	SerializedProperty showMinMaxHandleProp;
	SerializedProperty handleColorProp;
	SerializedProperty wireframeColorProp;

	// Private objects to hold primitive specific params temporarily
	SerializedObject planeParamsObj;
	SerializedObject cylinderParamsObj;
	SerializedObject coneParamsObj;
	SerializedObject sphereParamsObj;
	SerializedObject torusParamsObj;

	private int OnSceneGUICount = 0; //Debug
	private int OnInspectorGUICount = 0; //Debug

	public AnnexRComponentEditor(){
		//Debug.Log ("Contruct Editor");
	}

	void Awake(){
		//Debug.Log ("Awake Editor");
		OnSceneGUICount = 0; //Debug
		OnInspectorGUICount = 0; //Debug
		
		component = (AnnexRComponent)target;

		primitiveTypeProp = serializedObject.FindProperty("primitiveType");
		primitiveParamsProp = serializedObject.FindProperty("primitiveParams");
		primitivePrefProp = serializedObject.FindProperty("primitivePref");
		
		showWireframeProp = serializedObject.FindProperty("showWireframe");
		showHandlesProp = serializedObject.FindProperty("showHandles");
		showPositionHandleProp = serializedObject.FindProperty("showPositionHandle");
		showRotationHandleProp = serializedObject.FindProperty("showRotationHandle");
		showMinMaxHandleProp = serializedObject.FindProperty("showMinMaxHandle");
		handleColorProp = serializedObject.FindProperty ("handleColor");
		wireframeColorProp = serializedObject.FindProperty ("wireframeColor");

		planeParamsObj = new SerializedObject(serializedObject.FindProperty("planeParams").objectReferenceValue);
		cylinderParamsObj = new SerializedObject(serializedObject.FindProperty("cylinderParams").objectReferenceValue);
		coneParamsObj = new SerializedObject(serializedObject.FindProperty("coneParams").objectReferenceValue);
		sphereParamsObj = new SerializedObject(serializedObject.FindProperty("sphereParams").objectReferenceValue);
		torusParamsObj = new SerializedObject(serializedObject.FindProperty("torusParams").objectReferenceValue);
	}

	void OnEnable(){
		//Debug.Log ("Enable Editor");
	}

	void OnDisable(){
		//Debug.Log ("Disable Editor");
		Tools.hidden = false;
		EditorUtility.SetSelectedWireframeHidden (component.gameObject.GetComponent<Renderer> (), false);
	}

	void OnDestroy(){
		//Debug.Log ("Destroy Editor");
		//component.gameObject.tag = "Untagged";
		Tools.hidden = false;
		EditorUtility.SetSelectedWireframeHidden (component.gameObject.GetComponent<Renderer> (), false);
	}

	void OnSceneGUI(){
		if (component.enabled && showWireframeProp.boolValue) { // Draw custom handles on the object if component id enabled
			Tools.hidden = true;
			EditorUtility.SetSelectedWireframeHidden (component.gameObject.GetComponent<Renderer> (), true);
			serializedObject.Update ();

			switch (component.primitiveType) {
			case Shape.Plane:
				DrawPlaneHandles ();
				break;
			case Shape.Sphere:
				DrawSpherehandles ();
				break;
			case Shape.Cylinder:
				DrawCylinderHandles ();
				break;
			case Shape.Cone:
				DrawConeHandles ();
				break;
			case Shape.Torus:
				DrawTorushandles ();
				break;
			}
			serializedObject.ApplyModifiedProperties ();
			EditorUtility.SetDirty (target); // Account for child SerialisedObject changes which are invisible for serializedObject
		} else { // Else re-activate default handles
			Tools.hidden = false;
			EditorUtility.SetSelectedWireframeHidden (component.gameObject.GetComponent<Renderer> (), false);
		}
		//component.mprint("OnSceneGUI - " + (++OnSceneGUICount).ToString()); //Debug
	}

	public override void OnInspectorGUI(){
		if (component.enabled) { // Show inspector controls only if component is enabled
			//component.gameObject.tag = "AnnexingRealityPart";
			serializedObject.Update ();

			EditorGUILayout.BeginHorizontal();
			EditorGUIUtility.labelWidth =  GUI.skin.label.CalcSize(new GUIContent("Show Wireframe___")).x;
			EditorGUILayout.PropertyField (showWireframeProp, new GUIContent ("Show Wireframe", ""));
			if (GUILayout.Button(new GUIContent(Resources.Load("eye") as Texture, "Scaling handles"), GUILayout.Width(30), GUILayout.Height(25))){
				showHandlesProp.boolValue = true;
				showPositionHandleProp.boolValue = false;
				showRotationHandleProp.boolValue = false;
				showMinMaxHandleProp.boolValue = false;
			}
			//Texture2D move = (Texture2D)Resources.Load("move");
			//GUIStyle moveStyle = new GUIStyle();
			//moveStyle.normal.background = move;
			//moveStyle.active.background = move;
			if (GUILayout.Button(new GUIContent(Resources.Load("move") as Texture, "Translation handle"), GUILayout.Width(30), GUILayout.Height(25))){
				showHandlesProp.boolValue = false;
				showPositionHandleProp.boolValue = true;
				showRotationHandleProp.boolValue = false;
				showMinMaxHandleProp.boolValue = false;
			}
			if (GUILayout.Button(new GUIContent(Resources.Load("rotate") as Texture, "Rotation handle"), GUILayout.Width(30), GUILayout.Height(25))){
				showHandlesProp.boolValue = false;
				showPositionHandleProp.boolValue = false;
				showRotationHandleProp.boolValue = true;
				showMinMaxHandleProp.boolValue = false;
			}
			if (GUILayout.Button(new GUIContent(Resources.Load("minmax") as Texture, "Min max limits handles"), GUILayout.Width(30), GUILayout.Height(25))){
				showHandlesProp.boolValue = false;
				showPositionHandleProp.boolValue = false;
				showRotationHandleProp.boolValue = false;
				showMinMaxHandleProp.boolValue = true;
			}
			EditorGUILayout.EndHorizontal();

			EditorGUILayout.Separator();

			EditorGUILayout.BeginHorizontal();
			EditorGUIUtility.fieldWidth = 40;
			EditorGUIUtility.labelWidth =  GUI.skin.label.CalcSize(new GUIContent("Handle color____")).x;
			EditorGUILayout.PropertyField(handleColorProp, new GUIContent("Handle color"));
			EditorGUILayout.Space();
			EditorGUIUtility.labelWidth =  GUI.skin.label.CalcSize(new GUIContent("Wireframe color____")).x;
			EditorGUILayout.PropertyField(wireframeColorProp, new GUIContent("Wireframe color"));
			EditorGUILayout.EndHorizontal();

			EditorGUILayout.Space();

			EditorGUILayout.PropertyField (primitiveTypeProp, new GUIContent ("Primitive Shape"));

			switch (primitiveTypeProp.enumValueIndex) {
			case (int)Shape.Plane:
				//EditorGUILayout.Slider (primitivePrefProp, 0f, 1f, new GUIContent ("Shape Preference"));
				planeParamsObj.Update ();
				EditorGUILayout.LabelField("Planes are not supported. Please select a different primitive shape");
				//EditorGUILayout.PropertyField (planeParamsObj.FindProperty ("center"));
				//EditorGUILayout.PropertyField (planeParamsObj.FindProperty ("normal"));
				//EditorGUILayout.PropertyField (planeParamsObj.FindProperty ("width"));
				//EditorGUILayout.PropertyField (planeParamsObj.FindProperty ("length"));
				//EditorGUILayout.Slider (planeParamsObj.FindProperty ("widthPref"), 0f, 1f);
				//EditorGUILayout.Slider (planeParamsObj.FindProperty ("lengthPref"), 0f, 1f);
				//EditorGUILayout.Slider (planeParamsObj.FindProperty ("ratioPref"), 0f, 1f);
				planeParamsObj.ApplyModifiedProperties ();
				primitiveParamsProp.objectReferenceValue = planeParamsObj.targetObject;
				break;
			case (int)Shape.Sphere:
				EditorGUILayout.Slider (primitivePrefProp, 0f, 1f, new GUIContent ("Shape Priority"));
				sphereParamsObj.Update ();
				//EditorGUILayout.PropertyField (sphereParamsObj.FindProperty ("center"));
				EditorGUILayout.PropertyField (sphereParamsObj.FindProperty ("radius"), new GUIContent ("Radius (meters)"));
				EditorGUILayout.Slider (sphereParamsObj.FindProperty ("radiusPref"), 0f, 1f, new GUIContent ("Radius Priority"));
				//EditorGUILayout.Slider( "Orientation Priority", 0.5f, 0f, 1f);
				sphereParamsObj.ApplyModifiedProperties ();
				primitiveParamsProp.objectReferenceValue = sphereParamsObj.targetObject;
				break;
			case (int)Shape.Cylinder:
				EditorGUILayout.Slider (primitivePrefProp, 0f, 1f, new GUIContent ("Shape Priority"));
				cylinderParamsObj.Update ();
				//EditorGUILayout.PropertyField (cylinderParamsObj.FindProperty ("axisPosition"));
				//EditorGUILayout.PropertyField (cylinderParamsObj.FindProperty ("axisDirection"));
				EditorGUILayout.PropertyField (cylinderParamsObj.FindProperty ("radius"), new GUIContent ("Radius (meters)"));
				EditorGUILayout.PropertyField (cylinderParamsObj.FindProperty ("height"), new GUIContent ("Height (meters)"));
				EditorGUILayout.Slider (cylinderParamsObj.FindProperty ("radiusPref"), 0f, 1f, new GUIContent ("Radius Priority"));
				EditorGUILayout.Slider (cylinderParamsObj.FindProperty ("heightPref"), 0f, 1f, new GUIContent ("Height Priority"));
				EditorGUILayout.Slider (cylinderParamsObj.FindProperty ("ratioPref"), 0f, 1f, new GUIContent ("Ratio Priority"));
				//EditorGUILayout.Slider( "Orientation Preference", 0.5f, 0f, 1f);
				cylinderParamsObj.ApplyModifiedProperties ();
				primitiveParamsProp.objectReferenceValue = cylinderParamsObj.targetObject;
				break;
			case (int)Shape.Cone:
				EditorGUILayout.Slider (primitivePrefProp, 0f, 1f, new GUIContent ("Shape Priority"));
				coneParamsObj.Update ();
				//EditorGUILayout.PropertyField (coneParamsObj.FindProperty ("center"));
				//EditorGUILayout.PropertyField (coneParamsObj.FindProperty ("axisDirection"));
				EditorGUILayout.PropertyField (coneParamsObj.FindProperty ("angle"), new GUIContent ("Angle (degrees)"));
				EditorGUILayout.PropertyField (coneParamsObj.FindProperty ("height"), new GUIContent ("Height (meters)"));
				EditorGUILayout.Slider (coneParamsObj.FindProperty ("anglePref"), 0f, 1f, new GUIContent ("Angle Priority"));
				EditorGUILayout.Slider (coneParamsObj.FindProperty ("heightPref"), 0f, 1f, new GUIContent ("Height Priority"));
				EditorGUILayout.Slider (coneParamsObj.FindProperty ("ratioPref"), 0f, 1f, new GUIContent ("Ratio Priority"));
				//EditorGUILayout.Slider( "Orientation Preference", 0.5f, 0f, 1f);
				coneParamsObj.ApplyModifiedProperties ();
				primitiveParamsProp.objectReferenceValue = coneParamsObj.targetObject;
				break;
			case (int)Shape.Torus:
				EditorGUILayout.Slider (primitivePrefProp, 0f, 1f, new GUIContent ("Shape Priority"));
				torusParamsObj.Update ();
				//EditorGUILayout.PropertyField (torusParamsObj.FindProperty ("center"));
				//EditorGUILayout.PropertyField (torusParamsObj.FindProperty ("axisDirection"));
				EditorGUILayout.PropertyField (torusParamsObj.FindProperty ("minRadius"), new GUIContent ("Min Radius (meters)"));
				EditorGUILayout.PropertyField (torusParamsObj.FindProperty ("maxRadius"), new GUIContent ("Max Radius (meters)"));
				//EditorGUILayout.Slider (torusParamsObj.FindProperty ("minRadiusPref"), 0f, 1f);
				EditorGUILayout.Slider (torusParamsObj.FindProperty ("maxRadiusPref"), 0f, 1f, new GUIContent ("Max Rad Priority"));
				//EditorGUILayout.Slider (torusParamsObj.FindProperty ("ratioPref"), 0f, 1f);
				//EditorGUILayout.Slider( "Orientation Preference", 0.5f, 0f, 1f);
				torusParamsObj.ApplyModifiedProperties ();
				primitiveParamsProp.objectReferenceValue = torusParamsObj.targetObject;
				break;
			default:
				primitiveParamsProp.objectReferenceValue = (NoneParams)ScriptableObject.CreateInstance (typeof(NoneParams));
				break;
			}
			serializedObject.ApplyModifiedProperties ();
		} else {
			//component.gameObject.tag = "Untagged";
		}
		//component.mprint ("OnInspectorGUI - " + (++OnInspectorGUICount).ToString()); //Debug
	}

	private void DrawPlaneHandles(){
		Handles.matrix = component.transform.root.GetChild(3).transform.localToWorldMatrix;
		planeParamsObj.Update ();
		Vector3 localRightInWS = planeParamsObj.FindProperty("localRight").vector3Value,
				localUpInWS = planeParamsObj.FindProperty("localUp").vector3Value,
				localForwardInWS = -planeParamsObj.FindProperty("normal").vector3Value;
		Vector3 localOriginInWS = planeParamsObj.FindProperty("center").vector3Value;
		Vector3 worldHandlePtUL, worldHandlePtDR;
		Vector3 worldHandlePtULWidthMin, worldHandlePtDRWidthMin, worldHandlePtULLengthMin, worldHandlePtDRLengthMin;
		Vector3 worldHandlePtULWidthMax, worldHandlePtDRWidthMax, worldHandlePtULLengthMax, worldHandlePtDRLengthMax;

		Quaternion handleRotation = Quaternion.LookRotation (localForwardInWS, localUpInWS);
		
		// Rotation handle
		if(showRotationHandleProp.boolValue){
			handleRotation = Handles.RotationHandle (handleRotation, localOriginInWS);
			localRightInWS = handleRotation * Vector3.right;
			localUpInWS = handleRotation * Vector3.up;
			localForwardInWS = handleRotation * Vector3.forward;
			planeParamsObj.FindProperty ("localRight").vector3Value = localRightInWS;
			planeParamsObj.FindProperty ("localUp").vector3Value = localUpInWS;
			planeParamsObj.FindProperty ("normal").vector3Value = -localForwardInWS;
		}
		
		// Position handle
		if (showPositionHandleProp.boolValue) {
			localOriginInWS = Handles.PositionHandle (localOriginInWS, handleRotation);
			planeParamsObj.FindProperty ("center").vector3Value = localOriginInWS;
			
		}
		planeParamsObj.ApplyModifiedProperties ();

		float handleSize;

		Handles.color = handleColorProp.colorValue;

		if (showHandlesProp.boolValue) {
			// Upper Left handle
			planeParamsObj.Update ();
			worldHandlePtUL = localOriginInWS + 0.5f * planeParamsObj.FindProperty ("length").floatValue * localUpInWS -
				0.5f * planeParamsObj.FindProperty ("width").floatValue * localRightInWS;
			handleSize = HandleUtility.GetHandleSize (worldHandlePtUL) * 0.1f;
			worldHandlePtUL = Handles.FreeMoveHandle (worldHandlePtUL, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			planeParamsObj.FindProperty ("width").floatValue = Mathf.Max (0f, 2f * Vector3.Dot (worldHandlePtUL - localOriginInWS, -localRightInWS));
			planeParamsObj.FindProperty ("length").floatValue = Mathf.Max (0f, 2f * Vector3.Dot (worldHandlePtUL - localOriginInWS, localUpInWS));
			planeParamsObj.ApplyModifiedProperties ();

			// Down Right handle
			planeParamsObj.Update ();
			worldHandlePtDR = localOriginInWS - 0.5f * planeParamsObj.FindProperty ("length").floatValue * localUpInWS +
				0.5f * planeParamsObj.FindProperty ("width").floatValue * localRightInWS;
			worldHandlePtDR = Handles.FreeMoveHandle (worldHandlePtDR, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			planeParamsObj.FindProperty ("width").floatValue = Mathf.Max (0f, 2f * Vector3.Dot (worldHandlePtDR - localOriginInWS, localRightInWS));
			planeParamsObj.FindProperty ("length").floatValue = Mathf.Max (0f, 2f * Vector3.Dot (worldHandlePtDR - localOriginInWS, -localUpInWS));
			planeParamsObj.ApplyModifiedProperties ();
		}

		worldHandlePtUL = localOriginInWS + 0.5f * planeParamsObj.FindProperty ("length").floatValue * localUpInWS -
			0.5f * planeParamsObj.FindProperty ("width").floatValue * localRightInWS;
		worldHandlePtDR = localOriginInWS - 0.5f * planeParamsObj.FindProperty ("length").floatValue * localUpInWS +
			0.5f * planeParamsObj.FindProperty ("width").floatValue * localRightInWS;

		Vector3 worldHandlePtUR, worldHandlePtDL;
		float length = planeParamsObj.FindProperty ("length").floatValue;
		float width = planeParamsObj.FindProperty ("width").floatValue;
		float diagonal = 0.5f * Mathf.Sqrt (length * length + width * width);

		worldHandlePtUR = localOriginInWS + diagonal * (width * localRightInWS + length * localUpInWS).normalized;
		worldHandlePtDL = localOriginInWS + diagonal * (-width * localRightInWS -length * localUpInWS).normalized;

		float minMaxhandleSize;

		// MinMax handles
		if (showMinMaxHandleProp.boolValue) {
			planeParamsObj.Update ();
			worldHandlePtULWidthMin = localOriginInWS + 0.5f * planeParamsObj.FindProperty ("length").floatValue * localUpInWS -
				0.5f * planeParamsObj.FindProperty ("widthMin").floatValue * localRightInWS;
			worldHandlePtULWidthMax = localOriginInWS + 0.5f * planeParamsObj.FindProperty ("length").floatValue * localUpInWS -
				0.5f * planeParamsObj.FindProperty ("widthMax").floatValue * localRightInWS;
			worldHandlePtULLengthMin = localOriginInWS + 0.5f * planeParamsObj.FindProperty ("lengthMin").floatValue * localUpInWS -
				0.5f * planeParamsObj.FindProperty ("width").floatValue * localRightInWS;
			worldHandlePtULLengthMax = localOriginInWS + 0.5f * planeParamsObj.FindProperty ("lengthMax").floatValue * localUpInWS -
				0.5f * planeParamsObj.FindProperty ("width").floatValue * localRightInWS;
			minMaxhandleSize = HandleUtility.GetHandleSize (worldHandlePtULLengthMax) * 0.05f;
			worldHandlePtULWidthMin = Handles.FreeMoveHandle (worldHandlePtULWidthMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtULWidthMax = Handles.FreeMoveHandle (worldHandlePtULWidthMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtULLengthMin = Handles.FreeMoveHandle (worldHandlePtULLengthMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtULLengthMax = Handles.FreeMoveHandle (worldHandlePtULLengthMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			planeParamsObj.FindProperty ("widthMin").floatValue =
				Mathf.Min (Mathf.Max(0f, 2f * Vector3.Dot (worldHandlePtULWidthMin - localOriginInWS, -localRightInWS)), width);
			planeParamsObj.FindProperty ("widthMax").floatValue =
				Mathf.Max (2f * Vector3.Dot (worldHandlePtULWidthMax - localOriginInWS, -localRightInWS), width);
			planeParamsObj.FindProperty ("lengthMin").floatValue =
				Mathf.Min (Mathf.Max(0f, 2f * Vector3.Dot (worldHandlePtULLengthMin - localOriginInWS, localUpInWS)), length);
			planeParamsObj.FindProperty ("lengthMax").floatValue =
				Mathf.Max (2f * Vector3.Dot (worldHandlePtULLengthMax - localOriginInWS, localUpInWS), length);
			planeParamsObj.ApplyModifiedProperties ();

			planeParamsObj.Update ();
			worldHandlePtDRWidthMin = localOriginInWS - 0.5f * planeParamsObj.FindProperty ("length").floatValue * localUpInWS +
				0.5f * planeParamsObj.FindProperty ("widthMin").floatValue * localRightInWS;
			worldHandlePtDRWidthMax = localOriginInWS - 0.5f * planeParamsObj.FindProperty ("length").floatValue * localUpInWS +
				0.5f * planeParamsObj.FindProperty ("widthMax").floatValue * localRightInWS;
			worldHandlePtDRLengthMin = localOriginInWS - 0.5f * planeParamsObj.FindProperty ("lengthMin").floatValue * localUpInWS +
				0.5f * planeParamsObj.FindProperty ("width").floatValue * localRightInWS;
			worldHandlePtDRLengthMax = localOriginInWS - 0.5f * planeParamsObj.FindProperty ("lengthMax").floatValue * localUpInWS +
				0.5f * planeParamsObj.FindProperty ("width").floatValue * localRightInWS;
			worldHandlePtDRWidthMin = Handles.FreeMoveHandle (worldHandlePtDRWidthMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtDRWidthMax = Handles.FreeMoveHandle (worldHandlePtDRWidthMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtDRLengthMin = Handles.FreeMoveHandle (worldHandlePtDRLengthMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtDRLengthMax = Handles.FreeMoveHandle (worldHandlePtDRLengthMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			planeParamsObj.FindProperty ("widthMin").floatValue =
				Mathf.Min (Mathf.Max(0f, 2f * Vector3.Dot (worldHandlePtDRWidthMin - localOriginInWS, localRightInWS)), width);
			planeParamsObj.FindProperty ("widthMax").floatValue =
				Mathf.Max (2f * Vector3.Dot (worldHandlePtDRWidthMax - localOriginInWS, localRightInWS), width);
			planeParamsObj.FindProperty ("lengthMin").floatValue =
				Mathf.Min (Mathf.Max(0f, 2f * Vector3.Dot (worldHandlePtDRLengthMin - localOriginInWS, -localUpInWS)), length);
			planeParamsObj.FindProperty ("lengthMax").floatValue =
				Mathf.Max (2f * Vector3.Dot (worldHandlePtDRLengthMax - localOriginInWS, -localUpInWS), length);
			planeParamsObj.ApplyModifiedProperties ();
		
			// Draw minMaxGizmos
			drawMinMaxGizmo (worldHandlePtUL, worldHandlePtULWidthMax, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtUL, worldHandlePtULWidthMin, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtUL, worldHandlePtULLengthMin, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtUL, worldHandlePtULLengthMax, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtDR, worldHandlePtDRWidthMax, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtDR, worldHandlePtDRWidthMin, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtDR, worldHandlePtDRLengthMin, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtDR, worldHandlePtDRLengthMax, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);
		}

		// Draw wireframe
		Handles.color = wireframeColorProp.colorValue;
		Handles.DrawPolyLine(new Vector3[]{worldHandlePtUL, worldHandlePtUR, worldHandlePtDR, worldHandlePtDL, worldHandlePtUL});
		
	}

	private void DrawSpherehandles(){
		Handles.matrix = component.transform.root.GetChild(3).transform.localToWorldMatrix;
		sphereParamsObj.Update ();
		Vector3 localRightInWS = sphereParamsObj.FindProperty("localRight").vector3Value,
		localUpInWS = sphereParamsObj.FindProperty("localUp").vector3Value,
		localForwardInWS = sphereParamsObj.FindProperty("localForward").vector3Value;
		Vector3 localOriginInWS = sphereParamsObj.FindProperty("center").vector3Value;
		Vector3 worldHandlePtL, worldHandlePtR, worldHandlePtU, worldHandlePtD, worldHandlePtF, worldHandlePtB;
		Vector3 worldHandlePtUMin, worldHandlePtDMin, worldHandlePtLMin, worldHandlePtRMin, worldHandlePtFMin, worldHandlePtBMin;
		Vector3 worldHandlePtUMax, worldHandlePtDMax, worldHandlePtLMax, worldHandlePtRMax, worldHandlePtFMax, worldHandlePtBMax;

		Quaternion handleRotation = Quaternion.LookRotation (localForwardInWS, localUpInWS);
		
		// Rotation handle
		if(showRotationHandleProp.boolValue){
			handleRotation = Handles.RotationHandle (handleRotation, localOriginInWS);
			localRightInWS = handleRotation * Vector3.right;
			localUpInWS = handleRotation * Vector3.up;
			localForwardInWS = handleRotation * Vector3.forward;
			sphereParamsObj.FindProperty ("localRight").vector3Value = localRightInWS;
			sphereParamsObj.FindProperty ("localUp").vector3Value = localUpInWS;
			sphereParamsObj.FindProperty ("localForward").vector3Value = localForwardInWS;
		}
		
		// Position handle
		if (showPositionHandleProp.boolValue) {
			localOriginInWS = Handles.PositionHandle (localOriginInWS, handleRotation);
			sphereParamsObj.FindProperty ("center").vector3Value = localOriginInWS;
			
		}
		sphereParamsObj.ApplyModifiedProperties ();
		
		float handleSize;
		
		Handles.color = handleColorProp.colorValue;

		if (showHandlesProp.boolValue) {
			// Radius handles
			sphereParamsObj.Update ();
			worldHandlePtR = localOriginInWS + sphereParamsObj.FindProperty ("radius").floatValue * localRightInWS;
			handleSize = HandleUtility.GetHandleSize (worldHandlePtR) * 0.1f;
			worldHandlePtR = Handles.FreeMoveHandle (worldHandlePtR, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			sphereParamsObj.FindProperty ("radius").floatValue = Mathf.Max (0f, (worldHandlePtR - localOriginInWS).magnitude);
			sphereParamsObj.ApplyModifiedProperties ();
		
			sphereParamsObj.Update ();
			worldHandlePtL = localOriginInWS - sphereParamsObj.FindProperty ("radius").floatValue * localRightInWS;
			worldHandlePtL = Handles.FreeMoveHandle (worldHandlePtL, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			sphereParamsObj.FindProperty ("radius").floatValue = Mathf.Max (0f, (worldHandlePtL - localOriginInWS).magnitude);
			sphereParamsObj.ApplyModifiedProperties ();
		
			sphereParamsObj.Update ();
			worldHandlePtU = localOriginInWS + sphereParamsObj.FindProperty ("radius").floatValue * localUpInWS;
			worldHandlePtU = Handles.FreeMoveHandle (worldHandlePtU, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			sphereParamsObj.FindProperty ("radius").floatValue = Mathf.Max (0f, (worldHandlePtU - localOriginInWS).magnitude);
			sphereParamsObj.ApplyModifiedProperties ();
		
			sphereParamsObj.Update ();
			worldHandlePtD = localOriginInWS - sphereParamsObj.FindProperty ("radius").floatValue * localUpInWS;
			worldHandlePtD = Handles.FreeMoveHandle (worldHandlePtD, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			sphereParamsObj.FindProperty ("radius").floatValue = Mathf.Max (0f, (worldHandlePtD - localOriginInWS).magnitude);
			sphereParamsObj.ApplyModifiedProperties ();
		
			sphereParamsObj.Update ();
			worldHandlePtF = localOriginInWS + sphereParamsObj.FindProperty ("radius").floatValue * localForwardInWS;
			worldHandlePtF = Handles.FreeMoveHandle (worldHandlePtF, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			sphereParamsObj.FindProperty ("radius").floatValue = Mathf.Max (0f, (worldHandlePtF - localOriginInWS).magnitude);
			sphereParamsObj.ApplyModifiedProperties ();
		
			sphereParamsObj.Update ();
			worldHandlePtB = localOriginInWS - sphereParamsObj.FindProperty ("radius").floatValue * localForwardInWS;
			worldHandlePtB = Handles.FreeMoveHandle (worldHandlePtB, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			sphereParamsObj.FindProperty ("radius").floatValue = Mathf.Max (0f, (worldHandlePtB - localOriginInWS).magnitude);
			sphereParamsObj.ApplyModifiedProperties ();
		}

		worldHandlePtR = localOriginInWS + sphereParamsObj.FindProperty ("radius").floatValue * localRightInWS;
		worldHandlePtL = localOriginInWS - sphereParamsObj.FindProperty ("radius").floatValue * localRightInWS;
		worldHandlePtU = localOriginInWS + sphereParamsObj.FindProperty ("radius").floatValue * localUpInWS;
		worldHandlePtD = localOriginInWS - sphereParamsObj.FindProperty ("radius").floatValue * localUpInWS;
		worldHandlePtF = localOriginInWS + sphereParamsObj.FindProperty ("radius").floatValue * localForwardInWS;
		worldHandlePtB = localOriginInWS - sphereParamsObj.FindProperty ("radius").floatValue * localForwardInWS;

		float radius = (worldHandlePtL - localOriginInWS).magnitude;

		float minMaxhandleSize;

		// MinMax handles
		if (showMinMaxHandleProp.boolValue) {
			sphereParamsObj.Update();
			worldHandlePtRMin = localOriginInWS + sphereParamsObj.FindProperty("radiusMin").floatValue * localRightInWS;
			worldHandlePtRMax = localOriginInWS + sphereParamsObj.FindProperty("radiusMax").floatValue * localRightInWS;
			minMaxhandleSize = HandleUtility.GetHandleSize (worldHandlePtRMax) * 0.05f;
			worldHandlePtRMin = Handles.FreeMoveHandle (worldHandlePtRMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtRMax = Handles.FreeMoveHandle (worldHandlePtRMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			sphereParamsObj.FindProperty("radiusMin").floatValue = 
				Mathf.Min ((worldHandlePtRMin - localOriginInWS).magnitude, radius);
			sphereParamsObj.FindProperty("radiusMax").floatValue = 
				Mathf.Max ((worldHandlePtRMax - localOriginInWS).magnitude, radius);
			sphereParamsObj.ApplyModifiedProperties();

			sphereParamsObj.Update();
			worldHandlePtLMin = localOriginInWS - sphereParamsObj.FindProperty("radiusMin").floatValue * localRightInWS;
			worldHandlePtLMax = localOriginInWS - sphereParamsObj.FindProperty("radiusMax").floatValue * localRightInWS;
			worldHandlePtLMin = Handles.FreeMoveHandle (worldHandlePtLMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtLMax = Handles.FreeMoveHandle (worldHandlePtLMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			sphereParamsObj.FindProperty("radiusMin").floatValue = 
				Mathf.Min ((worldHandlePtLMin - localOriginInWS).magnitude, radius);
			sphereParamsObj.FindProperty("radiusMax").floatValue = 
				Mathf.Max ((worldHandlePtLMax - localOriginInWS).magnitude, radius);
			sphereParamsObj.ApplyModifiedProperties();

			sphereParamsObj.Update();
			worldHandlePtUMin = localOriginInWS + sphereParamsObj.FindProperty("radiusMin").floatValue * localUpInWS;
			worldHandlePtUMax = localOriginInWS + sphereParamsObj.FindProperty("radiusMax").floatValue * localUpInWS;
			worldHandlePtUMin = Handles.FreeMoveHandle (worldHandlePtUMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtUMax = Handles.FreeMoveHandle (worldHandlePtUMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			sphereParamsObj.FindProperty("radiusMin").floatValue = 
				Mathf.Min ((worldHandlePtUMin - localOriginInWS).magnitude, radius);
			sphereParamsObj.FindProperty("radiusMax").floatValue = 
				Mathf.Max ((worldHandlePtUMax - localOriginInWS).magnitude, radius);
			sphereParamsObj.ApplyModifiedProperties();

			sphereParamsObj.Update();
			worldHandlePtDMin = localOriginInWS - sphereParamsObj.FindProperty("radiusMin").floatValue * localUpInWS;
			worldHandlePtDMax = localOriginInWS - sphereParamsObj.FindProperty("radiusMax").floatValue * localUpInWS;
			worldHandlePtDMin = Handles.FreeMoveHandle (worldHandlePtDMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtDMax = Handles.FreeMoveHandle (worldHandlePtDMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			sphereParamsObj.FindProperty("radiusMin").floatValue = 
				Mathf.Min ((worldHandlePtDMin - localOriginInWS).magnitude, radius);
			sphereParamsObj.FindProperty("radiusMax").floatValue = 
				Mathf.Max ((worldHandlePtDMax - localOriginInWS).magnitude, radius);
			sphereParamsObj.ApplyModifiedProperties();

			sphereParamsObj.Update();
			worldHandlePtFMin = localOriginInWS + sphereParamsObj.FindProperty("radiusMin").floatValue * localForwardInWS;
			worldHandlePtFMax = localOriginInWS + sphereParamsObj.FindProperty("radiusMax").floatValue * localForwardInWS;
			worldHandlePtFMin = Handles.FreeMoveHandle (worldHandlePtFMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtFMax = Handles.FreeMoveHandle (worldHandlePtFMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			sphereParamsObj.FindProperty("radiusMin").floatValue = 
				Mathf.Min ((worldHandlePtFMin - localOriginInWS).magnitude, radius);
			sphereParamsObj.FindProperty("radiusMax").floatValue = 
				Mathf.Max ((worldHandlePtFMax - localOriginInWS).magnitude, radius);
			sphereParamsObj.ApplyModifiedProperties();

			sphereParamsObj.Update();
			worldHandlePtBMin = localOriginInWS - sphereParamsObj.FindProperty("radiusMin").floatValue * localForwardInWS;
			worldHandlePtBMax = localOriginInWS - sphereParamsObj.FindProperty("radiusMax").floatValue * localForwardInWS;
			worldHandlePtBMin = Handles.FreeMoveHandle (worldHandlePtBMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtBMax = Handles.FreeMoveHandle (worldHandlePtBMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			sphereParamsObj.FindProperty("radiusMin").floatValue = 
				Mathf.Min ((worldHandlePtBMin - localOriginInWS).magnitude, radius);
			sphereParamsObj.FindProperty("radiusMax").floatValue = 
				Mathf.Max ((worldHandlePtBMax - localOriginInWS).magnitude, radius);
			sphereParamsObj.ApplyModifiedProperties();
		
			// Draw minMaxGizmos
			drawMinMaxGizmo (worldHandlePtR, worldHandlePtRMin, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);	
			drawMinMaxGizmo (worldHandlePtR, worldHandlePtRMax, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);	
			drawMinMaxGizmo (worldHandlePtL, worldHandlePtLMin, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);	
			drawMinMaxGizmo (worldHandlePtL, worldHandlePtLMax, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);

			drawMinMaxGizmo (worldHandlePtU, worldHandlePtUMin, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtU, worldHandlePtUMax, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtD, worldHandlePtDMin, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtD, worldHandlePtDMax, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);

			drawMinMaxGizmo (worldHandlePtF, worldHandlePtFMin, localUpInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtF, worldHandlePtFMax, localUpInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtB, worldHandlePtBMin, localUpInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtB, worldHandlePtBMax, localUpInWS, localRightInWS, minMaxhandleSize * 4f);
		}

		// Draw wireframe
		Handles.color = wireframeColorProp.colorValue;
		Handles.DrawWireDisc (localOriginInWS, localRightInWS, radius);
		Handles.DrawWireDisc (localOriginInWS, localUpInWS, radius);
		Handles.DrawWireDisc (localOriginInWS, localForwardInWS, radius);
		Handles.DrawWireDisc (localOriginInWS, (localRightInWS + localForwardInWS).normalized, radius);
		Handles.DrawWireDisc (localOriginInWS, (localRightInWS - localForwardInWS).normalized, radius);

		handleSize = HandleUtility.GetHandleSize (worldHandlePtU) * 0.02f;
		Handles.DrawSolidDisc(worldHandlePtU, localUpInWS, handleSize);
	}

	private void DrawCylinderHandles(){
		Handles.matrix = component.transform.root.GetChild(3).transform.localToWorldMatrix;
		cylinderParamsObj.Update();
		Vector3 localRightInWS = cylinderParamsObj.FindProperty("localRight").vector3Value,
				localUpInWS = cylinderParamsObj.FindProperty("axisDirection").vector3Value,
				localForwardInWS = cylinderParamsObj.FindProperty("localForward").vector3Value;
		Vector3 localOriginInWS = cylinderParamsObj.FindProperty("axisPosition").vector3Value;
		Vector3 worldHandlePtU, worldHandlePtD, worldHandlePtL, worldHandlePtR, worldHandlePtF, worldHandlePtB;
		Vector3 worldHandlePtUMin, worldHandlePtDMin, worldHandlePtLMin, worldHandlePtRMin, worldHandlePtFMin, worldHandlePtBMin;
		Vector3 worldHandlePtUMax, worldHandlePtDMax, worldHandlePtLMax, worldHandlePtRMax, worldHandlePtFMax, worldHandlePtBMax;

		Quaternion handleRotation = Quaternion.LookRotation (localForwardInWS, localUpInWS);

		// Rotation handle
		if(showRotationHandleProp.boolValue){
			handleRotation = Handles.RotationHandle (handleRotation, localOriginInWS);
			localRightInWS = handleRotation * Vector3.right;
			localUpInWS = handleRotation * Vector3.up;
			localForwardInWS = handleRotation * Vector3.forward;
			cylinderParamsObj.FindProperty ("localRight").vector3Value = localRightInWS;
			cylinderParamsObj.FindProperty ("axisDirection").vector3Value = localUpInWS;
			cylinderParamsObj.FindProperty ("localForward").vector3Value = localForwardInWS;
		}

		// Position handle
		if (showPositionHandleProp.boolValue) {
			localOriginInWS = Handles.PositionHandle (localOriginInWS, handleRotation);
			cylinderParamsObj.FindProperty ("axisPosition").vector3Value = localOriginInWS;
		
		}
		cylinderParamsObj.ApplyModifiedProperties ();

		float handleSize;

		Handles.color = handleColorProp.colorValue;

		if (showHandlesProp.boolValue) {
			// Height handles
			cylinderParamsObj.Update ();
			worldHandlePtU = localOriginInWS + 0.5f * cylinderParamsObj.FindProperty ("height").floatValue * localUpInWS;
			handleSize = HandleUtility.GetHandleSize (worldHandlePtU) * 0.1f;
			worldHandlePtU = Handles.FreeMoveHandle (worldHandlePtU, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			cylinderParamsObj.FindProperty ("height").floatValue = Mathf.Max (0f, 2f * (worldHandlePtU - localOriginInWS).magnitude);
			cylinderParamsObj.ApplyModifiedProperties ();
		
			cylinderParamsObj.Update ();
			worldHandlePtD = localOriginInWS - 0.5f * cylinderParamsObj.FindProperty ("height").floatValue * localUpInWS;
			worldHandlePtD = Handles.FreeMoveHandle (worldHandlePtD, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			cylinderParamsObj.FindProperty ("height").floatValue = Mathf.Max (0f, 2f * (worldHandlePtD - localOriginInWS).magnitude);
			cylinderParamsObj.ApplyModifiedProperties ();

			// Radius handles
			cylinderParamsObj.Update ();
			worldHandlePtR = localOriginInWS + cylinderParamsObj.FindProperty ("radius").floatValue * localRightInWS;
			worldHandlePtR = Handles.FreeMoveHandle (worldHandlePtR, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			cylinderParamsObj.FindProperty ("radius").floatValue = Mathf.Max (0f, (worldHandlePtR - localOriginInWS).magnitude);
			cylinderParamsObj.ApplyModifiedProperties ();
		
			cylinderParamsObj.Update ();
			worldHandlePtL = localOriginInWS - cylinderParamsObj.FindProperty ("radius").floatValue * localRightInWS;
			worldHandlePtL = Handles.FreeMoveHandle (worldHandlePtL, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			cylinderParamsObj.FindProperty ("radius").floatValue = Mathf.Max (0f, (worldHandlePtL - localOriginInWS).magnitude);
			cylinderParamsObj.ApplyModifiedProperties ();
		
			cylinderParamsObj.Update ();
			worldHandlePtF = localOriginInWS + cylinderParamsObj.FindProperty ("radius").floatValue * localForwardInWS;
			worldHandlePtF = Handles.FreeMoveHandle (worldHandlePtF, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			cylinderParamsObj.FindProperty ("radius").floatValue = Mathf.Max (0f, (worldHandlePtF - localOriginInWS).magnitude);
			cylinderParamsObj.ApplyModifiedProperties ();
		
			cylinderParamsObj.Update ();
			worldHandlePtB = localOriginInWS - cylinderParamsObj.FindProperty ("radius").floatValue * localForwardInWS;
			worldHandlePtB = Handles.FreeMoveHandle (worldHandlePtB, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			cylinderParamsObj.FindProperty ("radius").floatValue = Mathf.Max (0f, (worldHandlePtB - localOriginInWS).magnitude);
			cylinderParamsObj.ApplyModifiedProperties ();
		}

		worldHandlePtU = localOriginInWS + 0.5f * cylinderParamsObj.FindProperty ("height").floatValue * localUpInWS;
		worldHandlePtD = localOriginInWS - 0.5f * cylinderParamsObj.FindProperty ("height").floatValue * localUpInWS;
		worldHandlePtR = localOriginInWS + cylinderParamsObj.FindProperty ("radius").floatValue * localRightInWS;
		worldHandlePtL = localOriginInWS - cylinderParamsObj.FindProperty ("radius").floatValue * localRightInWS;
		worldHandlePtF = localOriginInWS + cylinderParamsObj.FindProperty ("radius").floatValue * localForwardInWS;;
		worldHandlePtB = localOriginInWS - cylinderParamsObj.FindProperty ("radius").floatValue * localForwardInWS;

		float height = cylinderParamsObj.FindProperty ("height").floatValue;
		float radius = (worldHandlePtL - localOriginInWS).magnitude;

		float minMaxhandleSize;

		// MinMax handles
		if (showMinMaxHandleProp.boolValue) {
			cylinderParamsObj.Update();
			worldHandlePtUMin = localOriginInWS + 0.5f * cylinderParamsObj.FindProperty("heightMin").floatValue * localUpInWS;
			worldHandlePtUMax = localOriginInWS + 0.5f * cylinderParamsObj.FindProperty("heightMax").floatValue * localUpInWS;
			minMaxhandleSize =  HandleUtility.GetHandleSize (worldHandlePtUMax) * 0.05f;
			worldHandlePtUMin = Handles.FreeMoveHandle (worldHandlePtUMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtUMax = Handles.FreeMoveHandle (worldHandlePtUMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			cylinderParamsObj.FindProperty("heightMin").floatValue = 
				Mathf.Min (2f * (worldHandlePtUMin - localOriginInWS).magnitude, height);
			cylinderParamsObj.FindProperty("heightMax").floatValue = 
				Mathf.Max (2f * (worldHandlePtUMax - localOriginInWS).magnitude, height);
			cylinderParamsObj.ApplyModifiedProperties();

			cylinderParamsObj.Update();
			worldHandlePtDMin = localOriginInWS - 0.5f * cylinderParamsObj.FindProperty("heightMin").floatValue * localUpInWS;
			worldHandlePtDMax = localOriginInWS - 0.5f * cylinderParamsObj.FindProperty("heightMax").floatValue * localUpInWS;
			worldHandlePtDMin = Handles.FreeMoveHandle (worldHandlePtDMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtDMax = Handles.FreeMoveHandle (worldHandlePtDMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			cylinderParamsObj.FindProperty("heightMin").floatValue = 
				Mathf.Min (2f * (worldHandlePtDMin - localOriginInWS).magnitude, height);
			cylinderParamsObj.FindProperty("heightMax").floatValue = 
				Mathf.Max (2f * (worldHandlePtDMax - localOriginInWS).magnitude, height);
			cylinderParamsObj.ApplyModifiedProperties();

			cylinderParamsObj.Update();
			worldHandlePtRMin = localOriginInWS + cylinderParamsObj.FindProperty("radiusMin").floatValue * localRightInWS;
			worldHandlePtRMax = localOriginInWS + cylinderParamsObj.FindProperty("radiusMax").floatValue * localRightInWS;
			worldHandlePtRMin = Handles.FreeMoveHandle (worldHandlePtRMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtRMax = Handles.FreeMoveHandle (worldHandlePtRMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			cylinderParamsObj.FindProperty("radiusMin").floatValue = 
				Mathf.Min ((worldHandlePtRMin - localOriginInWS).magnitude, radius);
			cylinderParamsObj.FindProperty("radiusMax").floatValue = 
				Mathf.Max ((worldHandlePtRMax - localOriginInWS).magnitude, radius);
			cylinderParamsObj.ApplyModifiedProperties();

			cylinderParamsObj.Update();
			worldHandlePtLMin = localOriginInWS - cylinderParamsObj.FindProperty("radiusMin").floatValue * localRightInWS;
			worldHandlePtLMax = localOriginInWS - cylinderParamsObj.FindProperty("radiusMax").floatValue * localRightInWS;
			worldHandlePtLMin = Handles.FreeMoveHandle (worldHandlePtLMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtLMax = Handles.FreeMoveHandle (worldHandlePtLMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			cylinderParamsObj.FindProperty("radiusMin").floatValue = 
				Mathf.Min ((worldHandlePtLMin - localOriginInWS).magnitude, radius);
			cylinderParamsObj.FindProperty("radiusMax").floatValue = 
				Mathf.Max ((worldHandlePtLMax - localOriginInWS).magnitude, radius);
			cylinderParamsObj.ApplyModifiedProperties();

			cylinderParamsObj.Update();
			worldHandlePtFMin = localOriginInWS + cylinderParamsObj.FindProperty("radiusMin").floatValue * localForwardInWS;
			worldHandlePtFMax = localOriginInWS + cylinderParamsObj.FindProperty("radiusMax").floatValue * localForwardInWS;
			worldHandlePtFMin = Handles.FreeMoveHandle (worldHandlePtFMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtFMax = Handles.FreeMoveHandle (worldHandlePtFMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			cylinderParamsObj.FindProperty("radiusMin").floatValue = 
				Mathf.Min ((worldHandlePtFMin - localOriginInWS).magnitude, radius);
			cylinderParamsObj.FindProperty("radiusMax").floatValue = 
				Mathf.Max ((worldHandlePtFMax - localOriginInWS).magnitude, radius);
			cylinderParamsObj.ApplyModifiedProperties();

			cylinderParamsObj.Update();
			worldHandlePtBMin = localOriginInWS - cylinderParamsObj.FindProperty("radiusMin").floatValue * localForwardInWS;
			worldHandlePtBMax = localOriginInWS - cylinderParamsObj.FindProperty("radiusMax").floatValue * localForwardInWS;
			worldHandlePtBMin = Handles.FreeMoveHandle (worldHandlePtBMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtBMax = Handles.FreeMoveHandle (worldHandlePtBMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			cylinderParamsObj.FindProperty("radiusMin").floatValue = 
				Mathf.Min ((worldHandlePtBMin - localOriginInWS).magnitude, radius);
			cylinderParamsObj.FindProperty("radiusMax").floatValue = 
				Mathf.Max ((worldHandlePtBMax - localOriginInWS).magnitude, radius);
			cylinderParamsObj.ApplyModifiedProperties();
		
			// Draw minMaxGizmos
			drawMinMaxGizmo (worldHandlePtR, worldHandlePtRMin, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);	
			drawMinMaxGizmo (worldHandlePtR, worldHandlePtRMax, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);	
			drawMinMaxGizmo (worldHandlePtL, worldHandlePtLMin, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);	
			drawMinMaxGizmo (worldHandlePtL, worldHandlePtLMax, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);
			
			drawMinMaxGizmo (worldHandlePtU, worldHandlePtUMin, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtU, worldHandlePtUMax, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtD, worldHandlePtDMin, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtD, worldHandlePtDMax, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);
			
			drawMinMaxGizmo (worldHandlePtF, worldHandlePtFMin, localUpInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtF, worldHandlePtFMax, localUpInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtB, worldHandlePtBMin, localUpInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtB, worldHandlePtBMax, localUpInWS, localRightInWS, minMaxhandleSize * 4f);

		}
		
		// Draw wireframe
		Vector3 worldHandlePtUL = worldHandlePtL + 0.5f * height * localUpInWS;
		Vector3 worldHandlePtUR = worldHandlePtR + 0.5f * height * localUpInWS;
		Vector3 worldHandlePtUF = worldHandlePtF + 0.5f * height * localUpInWS;
		Vector3 worldHandlePtUB = worldHandlePtB + 0.5f * height * localUpInWS;
		Vector3 worldHandlePtDL = worldHandlePtL - 0.5f * height * localUpInWS;
		Vector3 worldHandlePtDR = worldHandlePtR - 0.5f * height* localUpInWS;
		Vector3 worldHandlePtDF = worldHandlePtF - 0.5f * height * localUpInWS;
		Vector3 worldHandlePtDB = worldHandlePtB - 0.5f * height * localUpInWS;

		Vector3 worldHandlePtULF = worldHandlePtU + radius * (-localRightInWS + localForwardInWS).normalized;
		Vector3 worldHandlePtURF = worldHandlePtU + radius * (localRightInWS + localForwardInWS).normalized;
		Vector3 worldHandlePtULB = worldHandlePtU + radius * (-localRightInWS - localForwardInWS).normalized;
		Vector3 worldHandlePtURB = worldHandlePtU + radius * (localRightInWS - localForwardInWS).normalized;
		Vector3 worldHandlePtDLF = worldHandlePtD + radius * (-localRightInWS + localForwardInWS).normalized;
		Vector3 worldHandlePtDRF = worldHandlePtD + radius * (localRightInWS + localForwardInWS).normalized;
		Vector3 worldHandlePtDLB = worldHandlePtD + radius * (-localRightInWS - localForwardInWS).normalized;
		Vector3 worldHandlePtDRB = worldHandlePtD + radius * (localRightInWS - localForwardInWS).normalized;
		
		Handles.color = wireframeColorProp.colorValue;
		Handles.DrawWireDisc (worldHandlePtU, localUpInWS, radius);
		Handles.DrawWireDisc (localOriginInWS, localUpInWS, radius);
		Handles.DrawWireDisc (worldHandlePtD, localUpInWS, radius);
		Handles.DrawLine (worldHandlePtUL, worldHandlePtDL);
		Handles.DrawLine (worldHandlePtUR, worldHandlePtDR);
		Handles.DrawLine (worldHandlePtUF, worldHandlePtDF);
		Handles.DrawLine (worldHandlePtUB, worldHandlePtDB);
		Handles.DrawLine (worldHandlePtULF, worldHandlePtDLF);
		Handles.DrawLine (worldHandlePtURF, worldHandlePtDRF);
		Handles.DrawLine (worldHandlePtULB, worldHandlePtDLB);
		Handles.DrawLine (worldHandlePtURB, worldHandlePtDRB);
		//Handles.DrawLine (worldHandlePtUL, worldHandlePtUR);
		//Handles.DrawLine (worldHandlePtUF, worldHandlePtUB);
		//Handles.DrawLine (worldHandlePtDL, worldHandlePtDR);
		//Handles.DrawLine (worldHandlePtDF, worldHandlePtDB);

		handleSize = HandleUtility.GetHandleSize (worldHandlePtU) * 0.02f;
		Handles.DrawSolidDisc(worldHandlePtU, localUpInWS, handleSize);
	}

	private void DrawConeHandles(){
		Handles.matrix = component.transform.root.GetChild(3).transform.localToWorldMatrix;
		coneParamsObj.Update ();
		Vector3 localRightInWS = coneParamsObj.FindProperty("localRight").vector3Value,
				localUpInWS = coneParamsObj.FindProperty("axisDirection").vector3Value,
				localForwardInWS = coneParamsObj.FindProperty("localForward").vector3Value;
		Vector3 localOriginInWS = coneParamsObj.FindProperty("center").vector3Value;
		Vector3 worldHandlePtU, worldHandlePtD, worldHandlePtL, worldHandlePtR, worldHandlePtF, worldHandlePtB;
		Vector3 worldHandlePtUMin, worldHandlePtDMin, worldHandlePtLMin, worldHandlePtRMin, worldHandlePtFMin, worldHandlePtBMin;
		Vector3 worldHandlePtUMax, worldHandlePtDMax, worldHandlePtLMax, worldHandlePtRMax, worldHandlePtFMax, worldHandlePtBMax;

		Quaternion handleRotation = Quaternion.LookRotation (localForwardInWS, localUpInWS);
		
		// Rotation handle
		if(showRotationHandleProp.boolValue){
			handleRotation = Handles.RotationHandle (handleRotation, localOriginInWS);
			localRightInWS = handleRotation * Vector3.right;
			localUpInWS = handleRotation * Vector3.up;
			localForwardInWS = handleRotation * Vector3.forward;
			coneParamsObj.FindProperty ("localRight").vector3Value = localRightInWS;
			coneParamsObj.FindProperty ("axisDirection").vector3Value = localUpInWS;
			coneParamsObj.FindProperty ("localForward").vector3Value = localForwardInWS;
		}
		
		// Position handle
		if (showPositionHandleProp.boolValue) {
			localOriginInWS = Handles.PositionHandle (localOriginInWS, handleRotation);
			coneParamsObj.FindProperty ("center").vector3Value = localOriginInWS;
			
		}
		coneParamsObj.ApplyModifiedProperties ();

		float handleSize;

		Handles.color = handleColorProp.colorValue;

		if (showHandlesProp.boolValue) {
			// Height handles
			coneParamsObj.Update ();
			worldHandlePtU = localOriginInWS + 0.5f * coneParamsObj.FindProperty ("height").floatValue * localUpInWS;
			handleSize = HandleUtility.GetHandleSize (worldHandlePtU) * 0.1f;
			worldHandlePtU = Handles.FreeMoveHandle (worldHandlePtU, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			coneParamsObj.FindProperty ("height").floatValue = Mathf.Max (0f, 2f * (worldHandlePtU - localOriginInWS).magnitude);
			coneParamsObj.ApplyModifiedProperties ();
		
			coneParamsObj.Update ();
			worldHandlePtD = localOriginInWS - 0.5f * coneParamsObj.FindProperty ("height").floatValue * localUpInWS;
			worldHandlePtD = Handles.FreeMoveHandle (worldHandlePtD, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			coneParamsObj.FindProperty ("height").floatValue = Mathf.Max (0f, 2f * (worldHandlePtD - localOriginInWS).magnitude);
			coneParamsObj.ApplyModifiedProperties ();
		}

		worldHandlePtU = localOriginInWS + 0.5f * coneParamsObj.FindProperty ("height").floatValue * localUpInWS;
		worldHandlePtD = localOriginInWS - 0.5f * coneParamsObj.FindProperty ("height").floatValue * localUpInWS;

		float height = coneParamsObj.FindProperty ("height").floatValue;
		float angle;

		// Angle (radius) handles
		if (height > 0) {
			if (showHandlesProp.boolValue) {
				coneParamsObj.Update ();
				angle = coneParamsObj.FindProperty ("angle").floatValue;
				worldHandlePtR = worldHandlePtD + height * Mathf.Tan (angle * Mathf.PI / 180f) * localRightInWS;
				handleSize = HandleUtility.GetHandleSize (worldHandlePtR) * 0.1f;
				worldHandlePtR = Handles.FreeMoveHandle (worldHandlePtR, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
				angle = Mathf.Atan (Mathf.Max (0f, (worldHandlePtR - worldHandlePtD).magnitude) / height) * 180f / Mathf.PI;
				coneParamsObj.FindProperty ("angle").floatValue = angle;
				coneParamsObj.ApplyModifiedProperties ();
			
				coneParamsObj.Update ();
				angle = coneParamsObj.FindProperty ("angle").floatValue;
				worldHandlePtL = worldHandlePtD - height * Mathf.Tan (angle * Mathf.PI / 180f) * localRightInWS;
				worldHandlePtL = Handles.FreeMoveHandle (worldHandlePtL, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
				angle = Mathf.Atan (Mathf.Max (0f, (worldHandlePtL - worldHandlePtD).magnitude) / height) * 180f / Mathf.PI;
				coneParamsObj.FindProperty ("angle").floatValue = angle;
				coneParamsObj.ApplyModifiedProperties ();

				coneParamsObj.Update ();
				angle = coneParamsObj.FindProperty ("angle").floatValue;
				worldHandlePtF = worldHandlePtD + height * Mathf.Tan (angle * Mathf.PI / 180f) * localForwardInWS;
				worldHandlePtF = Handles.FreeMoveHandle (worldHandlePtF, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
				angle = Mathf.Atan (Mathf.Max (0f, (worldHandlePtF - worldHandlePtD).magnitude) / height) * 180f / Mathf.PI;
				coneParamsObj.FindProperty ("angle").floatValue = angle;
				coneParamsObj.ApplyModifiedProperties ();

				coneParamsObj.Update ();
				angle = coneParamsObj.FindProperty ("angle").floatValue;
				worldHandlePtB = worldHandlePtD - height * Mathf.Tan (angle * Mathf.PI / 180f) * localForwardInWS;
				worldHandlePtB = Handles.FreeMoveHandle (worldHandlePtB, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
				angle = Mathf.Atan (Mathf.Max (0f, (worldHandlePtB - worldHandlePtD).magnitude) / height) * 180f / Mathf.PI;
				coneParamsObj.FindProperty ("angle").floatValue = angle;
				coneParamsObj.ApplyModifiedProperties ();
			}

			angle = coneParamsObj.FindProperty ("angle").floatValue;
			worldHandlePtR = worldHandlePtD + height * Mathf.Tan (angle * Mathf.PI / 180f) * localRightInWS;
			worldHandlePtL = worldHandlePtD - height * Mathf.Tan (angle * Mathf.PI / 180f) * localRightInWS;
			worldHandlePtF = worldHandlePtD + height * Mathf.Tan (angle * Mathf.PI / 180f) * localForwardInWS;
			worldHandlePtB = worldHandlePtD - height * Mathf.Tan (angle * Mathf.PI / 180f) * localForwardInWS;

			float radius = (worldHandlePtL - worldHandlePtD).magnitude;

			// Draw wireframe
			Vector3 worldHandlePtDLF = worldHandlePtD + radius * (-localRightInWS + localForwardInWS).normalized;
			Vector3 worldHandlePtDRF = worldHandlePtD + radius * (localRightInWS + localForwardInWS).normalized;
			Vector3 worldHandlePtDLB = worldHandlePtD + radius * (-localRightInWS - localForwardInWS).normalized;
			Vector3 worldHandlePtDRB = worldHandlePtD + radius * (localRightInWS - localForwardInWS).normalized;

			Handles.color = wireframeColorProp.colorValue;
			Handles.DrawWireDisc (worldHandlePtD, localUpInWS, radius);
			Handles.DrawLine (worldHandlePtU, worldHandlePtL);
			Handles.DrawLine (worldHandlePtU, worldHandlePtR);
			Handles.DrawLine (worldHandlePtU, worldHandlePtF);
			Handles.DrawLine (worldHandlePtU, worldHandlePtB);
			Handles.DrawLine (worldHandlePtU, worldHandlePtDLF);
			Handles.DrawLine (worldHandlePtU, worldHandlePtDRF);
			Handles.DrawLine (worldHandlePtU, worldHandlePtDLB);
			Handles.DrawLine (worldHandlePtU, worldHandlePtDRB);

			handleSize = HandleUtility.GetHandleSize (worldHandlePtU) * 0.02f;
			Handles.DrawSolidDisc(worldHandlePtU, localUpInWS, handleSize);
			//Handles.DrawLine (worldHandlePtL, worldHandlePtR);
			//Handles.DrawLine (worldHandlePtF, worldHandlePtB);
		}

		angle = coneParamsObj.FindProperty ("angle").floatValue;
		worldHandlePtR = worldHandlePtD + height * Mathf.Tan (angle * Mathf.PI / 180f) * localRightInWS;
		worldHandlePtL = worldHandlePtD - height * Mathf.Tan (angle * Mathf.PI / 180f) * localRightInWS;
		worldHandlePtF = worldHandlePtD + height * Mathf.Tan (angle * Mathf.PI / 180f) * localForwardInWS;
		worldHandlePtB = worldHandlePtD - height * Mathf.Tan (angle * Mathf.PI / 180f) * localForwardInWS;

		float heightMin, heightMax;
		float angleMin, angleMax;

		Handles.color = handleColorProp.colorValue;

		float minMaxhandleSize;

		// MinMax handles
		if (showMinMaxHandleProp.boolValue) {
			coneParamsObj.Update();
			worldHandlePtUMin = localOriginInWS + 0.5f * coneParamsObj.FindProperty("heightMin").floatValue * localUpInWS;
			worldHandlePtUMax = localOriginInWS + 0.5f * coneParamsObj.FindProperty("heightMax").floatValue * localUpInWS;
			minMaxhandleSize = HandleUtility.GetHandleSize (worldHandlePtUMax) * 0.05f;
			worldHandlePtUMin = Handles.FreeMoveHandle (worldHandlePtUMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtUMax = Handles.FreeMoveHandle (worldHandlePtUMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			coneParamsObj.FindProperty("heightMin").floatValue = 
				Mathf.Min (2f * (worldHandlePtUMin - localOriginInWS).magnitude, height);
			coneParamsObj.FindProperty("heightMax").floatValue = 
				Mathf.Max (2f * (worldHandlePtUMax - localOriginInWS).magnitude, height);
			coneParamsObj.ApplyModifiedProperties ();

			coneParamsObj.Update();
			worldHandlePtDMin = localOriginInWS - 0.5f * coneParamsObj.FindProperty("heightMin").floatValue * localUpInWS;
			worldHandlePtDMax = localOriginInWS - 0.5f * coneParamsObj.FindProperty("heightMax").floatValue * localUpInWS;
			worldHandlePtDMin = Handles.FreeMoveHandle (worldHandlePtDMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtDMax = Handles.FreeMoveHandle (worldHandlePtDMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			coneParamsObj.FindProperty("heightMin").floatValue = 
				Mathf.Min (2f * (worldHandlePtDMin - localOriginInWS).magnitude, height);
			coneParamsObj.FindProperty("heightMax").floatValue = 
				Mathf.Max (2f * (worldHandlePtDMax - localOriginInWS).magnitude, height);
			coneParamsObj.ApplyModifiedProperties ();

			heightMin = coneParamsObj.FindProperty("heightMin").floatValue;
			heightMax = coneParamsObj.FindProperty("heightMax").floatValue;

			// Draw minMaxGizmos
			drawMinMaxGizmo (worldHandlePtU, worldHandlePtUMin, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtU, worldHandlePtUMax, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtD, worldHandlePtDMin, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtD, worldHandlePtDMax, localForwardInWS, localRightInWS, minMaxhandleSize * 4f);

			if(heightMin > 0){
				coneParamsObj.Update ();
				angleMin = coneParamsObj.FindProperty ("angleMin").floatValue;
				angleMax = coneParamsObj.FindProperty ("angleMax").floatValue;
				worldHandlePtRMin = worldHandlePtD + heightMin * Mathf.Tan (angleMin * Mathf.PI / 180f) * localRightInWS;
				worldHandlePtRMax = worldHandlePtD + heightMax * Mathf.Tan (angleMax * Mathf.PI / 180f) * localRightInWS;
				worldHandlePtRMin = Handles.FreeMoveHandle (worldHandlePtRMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
				worldHandlePtRMax = Handles.FreeMoveHandle (worldHandlePtRMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
				angleMin = Mathf.Min(Mathf.Atan ((worldHandlePtRMin - worldHandlePtD).magnitude / heightMin) * 180f / Mathf.PI, angle);
				angleMax = Mathf.Max(Mathf.Atan ((worldHandlePtRMax - worldHandlePtD).magnitude / heightMax) * 180f / Mathf.PI, angle);
				coneParamsObj.FindProperty ("angleMin").floatValue = angleMin;
				coneParamsObj.FindProperty ("angleMax").floatValue = angleMax;
				coneParamsObj.ApplyModifiedProperties ();

				coneParamsObj.Update ();
				angleMin = coneParamsObj.FindProperty ("angleMin").floatValue;
				angleMax = coneParamsObj.FindProperty ("angleMax").floatValue;
				worldHandlePtLMin = worldHandlePtD - heightMin * Mathf.Tan (angleMin * Mathf.PI / 180f) * localRightInWS;
				worldHandlePtLMax = worldHandlePtD - heightMax * Mathf.Tan (angleMax * Mathf.PI / 180f) * localRightInWS;
				worldHandlePtLMin = Handles.FreeMoveHandle (worldHandlePtLMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
				worldHandlePtLMax = Handles.FreeMoveHandle (worldHandlePtLMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
				angleMin = Mathf.Min(Mathf.Atan ((worldHandlePtLMin - worldHandlePtD).magnitude / heightMin) * 180f / Mathf.PI, angle);
				angleMax = Mathf.Max(Mathf.Atan ((worldHandlePtLMax - worldHandlePtD).magnitude / heightMax) * 180f / Mathf.PI, angle);
				coneParamsObj.FindProperty ("angleMin").floatValue = angleMin;
				coneParamsObj.FindProperty ("angleMax").floatValue = angleMax;
				coneParamsObj.ApplyModifiedProperties ();

				coneParamsObj.Update ();
				angleMin = coneParamsObj.FindProperty ("angleMin").floatValue;
				angleMax = coneParamsObj.FindProperty ("angleMax").floatValue;
				worldHandlePtFMin = worldHandlePtD + heightMin * Mathf.Tan (angleMin * Mathf.PI / 180f) * localForwardInWS;
				worldHandlePtFMax = worldHandlePtD + heightMax * Mathf.Tan (angleMax * Mathf.PI / 180f) * localForwardInWS;
				worldHandlePtFMin = Handles.FreeMoveHandle (worldHandlePtFMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
				worldHandlePtFMax = Handles.FreeMoveHandle (worldHandlePtFMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
				angleMin = Mathf.Min(Mathf.Atan ((worldHandlePtFMin - worldHandlePtD).magnitude / heightMin) * 180f / Mathf.PI, angle);
				angleMax = Mathf.Max(Mathf.Atan ((worldHandlePtFMax - worldHandlePtD).magnitude / heightMax) * 180f / Mathf.PI, angle);
				coneParamsObj.FindProperty ("angleMin").floatValue = angleMin;
				coneParamsObj.FindProperty ("angleMax").floatValue = angleMax;
				coneParamsObj.ApplyModifiedProperties ();

				coneParamsObj.Update ();
				angleMin = coneParamsObj.FindProperty ("angleMin").floatValue;
				angleMax = coneParamsObj.FindProperty ("angleMax").floatValue;
				worldHandlePtBMin = worldHandlePtD - heightMin * Mathf.Tan (angleMin * Mathf.PI / 180f) * localForwardInWS;
				worldHandlePtBMax = worldHandlePtD - heightMax * Mathf.Tan (angleMax * Mathf.PI / 180f) * localForwardInWS;
				worldHandlePtBMin = Handles.FreeMoveHandle (worldHandlePtBMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
				worldHandlePtBMax = Handles.FreeMoveHandle (worldHandlePtBMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
				angleMin = Mathf.Min(Mathf.Atan ((worldHandlePtBMin - worldHandlePtD).magnitude / heightMin) * 180f / Mathf.PI, angle);
				angleMax = Mathf.Max(Mathf.Atan ((worldHandlePtBMax - worldHandlePtD).magnitude / heightMax) * 180f / Mathf.PI, angle);
				coneParamsObj.FindProperty ("angleMin").floatValue = angleMin;
				coneParamsObj.FindProperty ("angleMax").floatValue = angleMax;
				coneParamsObj.ApplyModifiedProperties ();
			
				// Draw minMaxGizmos
				drawMinMaxGizmo (worldHandlePtR, worldHandlePtRMin, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);	
				drawMinMaxGizmo (worldHandlePtR, worldHandlePtRMax, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);	
				drawMinMaxGizmo (worldHandlePtL, worldHandlePtLMin, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);	
				drawMinMaxGizmo (worldHandlePtL, worldHandlePtLMax, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);

				drawMinMaxGizmo (worldHandlePtF, worldHandlePtFMin, localUpInWS, localRightInWS, minMaxhandleSize * 4f);
				drawMinMaxGizmo (worldHandlePtF, worldHandlePtFMax, localUpInWS, localRightInWS, minMaxhandleSize * 4f);
				drawMinMaxGizmo (worldHandlePtB, worldHandlePtBMin, localUpInWS, localRightInWS, minMaxhandleSize * 4f);
				drawMinMaxGizmo (worldHandlePtB, worldHandlePtBMax, localUpInWS, localRightInWS, minMaxhandleSize * 4f);
			}
		}
	}

	private void DrawTorushandles(){
		Handles.matrix = component.transform.root.GetChild(3).transform.localToWorldMatrix;
		torusParamsObj.Update ();
		Vector3 localRightInWS = torusParamsObj.FindProperty("localRight").vector3Value,
				localUpInWS = torusParamsObj.FindProperty("axisDirection").vector3Value,
				localForwardInWS = torusParamsObj.FindProperty("localForward").vector3Value;
		Vector3 localOriginInWS = torusParamsObj.FindProperty("center").vector3Value;
		Vector3 worldHandlePtLMin, worldHandlePtRMin, worldHandlePtFMin, worldHandlePtBMin;
		Vector3 worldHandlePtLMax, worldHandlePtRMax, worldHandlePtFMax, worldHandlePtBMax;

		Vector3 worldHandlePtLMaxMin, worldHandlePtRMaxMin, worldHandlePtFMaxMin, worldHandlePtBMaxMin;
		Vector3 worldHandlePtLMaxMax, worldHandlePtRMaxMax, worldHandlePtFMaxMax, worldHandlePtBMaxMax;

		Quaternion handleRotation = Quaternion.LookRotation (localForwardInWS, localUpInWS);
		
		// Rotation handle
		if(showRotationHandleProp.boolValue){
			handleRotation = Handles.RotationHandle (handleRotation, localOriginInWS);
			localRightInWS = handleRotation * Vector3.right;
			localUpInWS = handleRotation * Vector3.up;
			localForwardInWS = handleRotation * Vector3.forward;
			torusParamsObj.FindProperty ("localRight").vector3Value = localRightInWS;
			torusParamsObj.FindProperty ("axisDirection").vector3Value = localUpInWS;
			torusParamsObj.FindProperty ("localForward").vector3Value = localForwardInWS;
		}
		
		// Position handle
		if (showPositionHandleProp.boolValue) {
			localOriginInWS = Handles.PositionHandle (localOriginInWS, handleRotation);
			torusParamsObj.FindProperty ("center").vector3Value = localOriginInWS;
			
		}
		torusParamsObj.ApplyModifiedProperties ();

		float handleSize;

		Handles.color = handleColorProp.colorValue;

		if (showHandlesProp.boolValue) {
			// Min radius handles
			torusParamsObj.Update ();
			worldHandlePtRMin = localOriginInWS + torusParamsObj.FindProperty ("minRadius").floatValue * localRightInWS;
			handleSize = HandleUtility.GetHandleSize (worldHandlePtRMin) * 0.1f;
			worldHandlePtRMin = Handles.FreeMoveHandle (worldHandlePtRMin, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			torusParamsObj.FindProperty ("minRadius").floatValue = Mathf.Max (0f, (worldHandlePtRMin - localOriginInWS).magnitude);
			torusParamsObj.ApplyModifiedProperties ();
		
			torusParamsObj.Update ();
			worldHandlePtLMin = localOriginInWS - torusParamsObj.FindProperty ("minRadius").floatValue * localRightInWS;
			worldHandlePtLMin = Handles.FreeMoveHandle (worldHandlePtLMin, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			torusParamsObj.FindProperty ("minRadius").floatValue = Mathf.Max (0f, (worldHandlePtLMin - localOriginInWS).magnitude);
			torusParamsObj.ApplyModifiedProperties ();

			torusParamsObj.Update ();
			worldHandlePtFMin = localOriginInWS + torusParamsObj.FindProperty ("minRadius").floatValue * localForwardInWS;
			worldHandlePtFMin = Handles.FreeMoveHandle (worldHandlePtFMin, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			torusParamsObj.FindProperty ("minRadius").floatValue = Mathf.Max (0f, (worldHandlePtFMin - localOriginInWS).magnitude);
			torusParamsObj.ApplyModifiedProperties ();
		
			torusParamsObj.Update ();
			worldHandlePtBMin = localOriginInWS - torusParamsObj.FindProperty ("minRadius").floatValue * localForwardInWS;
			worldHandlePtBMin = Handles.FreeMoveHandle (worldHandlePtBMin, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			torusParamsObj.FindProperty ("minRadius").floatValue = Mathf.Max (0f, (worldHandlePtBMin - localOriginInWS).magnitude);
			torusParamsObj.ApplyModifiedProperties ();
		}

		worldHandlePtRMin = localOriginInWS + torusParamsObj.FindProperty ("minRadius").floatValue * localRightInWS;
		worldHandlePtLMin = localOriginInWS - torusParamsObj.FindProperty ("minRadius").floatValue * localRightInWS;
		worldHandlePtFMin = localOriginInWS + torusParamsObj.FindProperty ("minRadius").floatValue * localForwardInWS;
		worldHandlePtBMin = localOriginInWS - torusParamsObj.FindProperty ("minRadius").floatValue * localForwardInWS;

		float minRadius = torusParamsObj.FindProperty("minRadius").floatValue;

		if (showHandlesProp.boolValue) {
			// Maximum radius handles
			torusParamsObj.Update ();
			worldHandlePtRMax = localOriginInWS + torusParamsObj.FindProperty ("maxRadius").floatValue * localRightInWS;
			handleSize = HandleUtility.GetHandleSize (worldHandlePtRMax) * 0.1f;
			worldHandlePtRMax = Handles.FreeMoveHandle (worldHandlePtRMax, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			torusParamsObj.FindProperty ("maxRadius").floatValue = Mathf.Max (minRadius, (worldHandlePtRMax - localOriginInWS).magnitude);
			torusParamsObj.ApplyModifiedProperties ();
		
			torusParamsObj.Update ();
			worldHandlePtLMax = localOriginInWS - torusParamsObj.FindProperty ("maxRadius").floatValue * localRightInWS;
			worldHandlePtLMax = Handles.FreeMoveHandle (worldHandlePtLMax, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			torusParamsObj.FindProperty ("maxRadius").floatValue = Mathf.Max (minRadius, (worldHandlePtLMax - localOriginInWS).magnitude);
			torusParamsObj.ApplyModifiedProperties ();
		
			torusParamsObj.Update ();
			worldHandlePtFMax = localOriginInWS + torusParamsObj.FindProperty ("maxRadius").floatValue * localForwardInWS;
			worldHandlePtFMax = Handles.FreeMoveHandle (worldHandlePtFMax, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			torusParamsObj.FindProperty ("maxRadius").floatValue = Mathf.Max (minRadius, (worldHandlePtFMax - localOriginInWS).magnitude);
			torusParamsObj.ApplyModifiedProperties ();
		
			torusParamsObj.Update ();
			worldHandlePtBMax = localOriginInWS - torusParamsObj.FindProperty ("maxRadius").floatValue * localForwardInWS;
			worldHandlePtBMax = Handles.FreeMoveHandle (worldHandlePtBMax, Quaternion.identity, handleSize, Vector3.zero, Handles.SphereCap);
			torusParamsObj.FindProperty ("maxRadius").floatValue = Mathf.Max (minRadius, (worldHandlePtBMax - localOriginInWS).magnitude);
			torusParamsObj.ApplyModifiedProperties ();
		}

		worldHandlePtRMax = localOriginInWS + torusParamsObj.FindProperty ("maxRadius").floatValue * localRightInWS;
		worldHandlePtLMax = localOriginInWS - torusParamsObj.FindProperty ("maxRadius").floatValue * localRightInWS;
		worldHandlePtFMax = localOriginInWS + torusParamsObj.FindProperty ("maxRadius").floatValue * localForwardInWS;
		worldHandlePtBMax = localOriginInWS - torusParamsObj.FindProperty ("maxRadius").floatValue * localForwardInWS;

		float maxRadius = torusParamsObj.FindProperty("maxRadius").floatValue;
		float torusRadius = 0.5f * (maxRadius - minRadius);
		float midRadius = 0.5f * (maxRadius + minRadius);

		float minMaxhandleSize;

		// MinMax handles
		if (showMinMaxHandleProp.boolValue) {
			torusParamsObj.Update();
			worldHandlePtRMaxMin = localOriginInWS + torusParamsObj.FindProperty("maxRadiusMin").floatValue * localRightInWS;
			worldHandlePtRMaxMax = localOriginInWS + torusParamsObj.FindProperty("maxRadiusMax").floatValue * localRightInWS;
			minMaxhandleSize = HandleUtility.GetHandleSize (worldHandlePtRMaxMax) * 0.05f;
			worldHandlePtRMaxMin = Handles.FreeMoveHandle (worldHandlePtRMaxMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtRMaxMax = Handles.FreeMoveHandle (worldHandlePtRMaxMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			torusParamsObj.FindProperty("maxRadiusMin").floatValue = 
				Mathf.Min (Mathf.Max(minRadius, (worldHandlePtRMaxMin - localOriginInWS).magnitude), maxRadius);
			torusParamsObj.FindProperty("maxRadiusMax").floatValue = 
				Mathf.Max (maxRadius, (worldHandlePtRMaxMax - localOriginInWS).magnitude);
			torusParamsObj.ApplyModifiedProperties();

			torusParamsObj.Update();
			worldHandlePtLMaxMin = localOriginInWS - torusParamsObj.FindProperty("maxRadiusMin").floatValue * localRightInWS;
			worldHandlePtLMaxMax = localOriginInWS - torusParamsObj.FindProperty("maxRadiusMax").floatValue * localRightInWS;
			worldHandlePtLMaxMin = Handles.FreeMoveHandle (worldHandlePtLMaxMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtLMaxMax = Handles.FreeMoveHandle (worldHandlePtLMaxMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			torusParamsObj.FindProperty("maxRadiusMin").floatValue = 
				Mathf.Min (Mathf.Max(minRadius, (worldHandlePtLMaxMin - localOriginInWS).magnitude), maxRadius);
			torusParamsObj.FindProperty("maxRadiusMax").floatValue = 
				Mathf.Max (maxRadius, (worldHandlePtLMaxMax - localOriginInWS).magnitude);
			torusParamsObj.ApplyModifiedProperties();

			torusParamsObj.Update();
			worldHandlePtFMaxMin = localOriginInWS + torusParamsObj.FindProperty("maxRadiusMin").floatValue * localForwardInWS;
			worldHandlePtFMaxMax = localOriginInWS + torusParamsObj.FindProperty("maxRadiusMax").floatValue * localForwardInWS;
			worldHandlePtFMaxMin = Handles.FreeMoveHandle (worldHandlePtFMaxMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtFMaxMax = Handles.FreeMoveHandle (worldHandlePtFMaxMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			torusParamsObj.FindProperty("maxRadiusMin").floatValue = 
				Mathf.Min (Mathf.Max(minRadius, (worldHandlePtFMaxMin - localOriginInWS).magnitude), maxRadius);
			torusParamsObj.FindProperty("maxRadiusMax").floatValue = 
				Mathf.Max (maxRadius, (worldHandlePtFMaxMax - localOriginInWS).magnitude);
			torusParamsObj.ApplyModifiedProperties();

			torusParamsObj.Update();
			worldHandlePtBMaxMin = localOriginInWS - torusParamsObj.FindProperty("maxRadiusMin").floatValue * localForwardInWS;
			worldHandlePtBMaxMax = localOriginInWS - torusParamsObj.FindProperty("maxRadiusMax").floatValue * localForwardInWS;
			worldHandlePtBMaxMin = Handles.FreeMoveHandle (worldHandlePtBMaxMin, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			worldHandlePtBMaxMax = Handles.FreeMoveHandle (worldHandlePtBMaxMax, Quaternion.identity, minMaxhandleSize, Vector3.zero, Handles.CubeCap);
			torusParamsObj.FindProperty("maxRadiusMin").floatValue = 
				Mathf.Min (Mathf.Max(minRadius, (worldHandlePtBMaxMin - localOriginInWS).magnitude), maxRadius);
			torusParamsObj.FindProperty("maxRadiusMax").floatValue = 
				Mathf.Max (maxRadius, (worldHandlePtBMaxMax - localOriginInWS).magnitude);
			torusParamsObj.ApplyModifiedProperties();

			// Draw minMaxGizmos
			drawMinMaxGizmo (worldHandlePtRMax, worldHandlePtRMaxMin, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);	
			drawMinMaxGizmo (worldHandlePtRMax, worldHandlePtRMaxMax, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);	
			drawMinMaxGizmo (worldHandlePtLMax, worldHandlePtLMaxMin, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);	
			drawMinMaxGizmo (worldHandlePtLMax, worldHandlePtLMaxMax, localUpInWS, localForwardInWS, minMaxhandleSize * 4f);
			
			drawMinMaxGizmo (worldHandlePtFMax, worldHandlePtFMaxMin, localUpInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtFMax, worldHandlePtFMaxMax, localUpInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtBMax, worldHandlePtBMaxMin, localUpInWS, localRightInWS, minMaxhandleSize * 4f);
			drawMinMaxGizmo (worldHandlePtBMax, worldHandlePtBMaxMax, localUpInWS, localRightInWS, minMaxhandleSize * 4f);
		}

		// Draw wireframe
		Handles.color = wireframeColorProp.colorValue;
		Handles.DrawWireDisc (localOriginInWS, localUpInWS, minRadius);
		Handles.DrawWireDisc (localOriginInWS, localUpInWS, maxRadius);
		Handles.DrawWireDisc (localOriginInWS + torusRadius * localUpInWS, localUpInWS, 0.5f *(minRadius + maxRadius));
		Handles.DrawWireDisc (localOriginInWS - torusRadius * localUpInWS, localUpInWS, 0.5f *(minRadius + maxRadius));
		Handles.DrawWireDisc (localOriginInWS + midRadius * localRightInWS, localForwardInWS, torusRadius);
		Handles.DrawWireDisc (localOriginInWS - midRadius * localRightInWS, localForwardInWS, torusRadius);
		Handles.DrawWireDisc (localOriginInWS + midRadius * localForwardInWS, localRightInWS, torusRadius);
		Handles.DrawWireDisc (localOriginInWS - midRadius * localForwardInWS, localRightInWS, torusRadius);

		//Vector3 localRightForwardInWS = (localRightInWS + localForwardInWS).normalized;
		//Vector3 localRightBackInWS = (localRightInWS - localForwardInWS).normalized;

		//Handles.DrawWireDisc (center + midRadius * localRightForwardInWS, localRightBackInWS, torusRadius);
		//Handles.DrawWireDisc (center - midRadius * localRightForwardInWS, localRightBackInWS, torusRadius);
		//Handles.DrawWireDisc (center + midRadius * localRightBackInWS, localRightForwardInWS, torusRadius);
		//Handles.DrawWireDisc (center - midRadius * localRightBackInWS, localRightForwardInWS, torusRadius);

		Vector3 localUpPt = localOriginInWS + torusRadius * localUpInWS;
		handleSize = HandleUtility.GetHandleSize (localUpPt) * 0.02f;
		Handles.DrawSolidDisc(localUpPt, localUpInWS, handleSize);
	}

	void drawMinMaxGizmo(Vector3 beginPt, Vector3 endPt, Vector3 vertDir, Vector3 horizDir, float size){
		Vector3 uPt, dPt, rPt, lPt;

		uPt = endPt + size * vertDir;
		dPt = endPt - size * vertDir;
		rPt = endPt + size * horizDir;
		lPt = endPt - size * horizDir;

		Handles.color = handleColorProp.colorValue;

		Handles.DrawLine (beginPt, endPt);
		Handles.DrawLine (uPt, dPt);
		Handles.DrawLine (rPt, lPt);
	}
}
