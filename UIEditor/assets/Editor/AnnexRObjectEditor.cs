using UnityEngine;
using UnityEditor;
using System.Collections;
using AnnexR;

[CustomEditor(typeof(AnnexRObject))]
public class AnnexRObjectEditor : Editor {
	// Object component instance this editor belongs to. Read only
	AnnexRObject component;

	SerializedProperty objectPrefProp;
	SerializedProperty allowJumpingProp;

	void Awake(){
		component = (AnnexRObject)target;
		component.gameObject.tag = "AnnexingRealityObject";
		objectPrefProp = serializedObject.FindProperty ("objectPref");
		allowJumpingProp = serializedObject.FindProperty ("allowJumping");
	}

	public override void OnInspectorGUI(){
		serializedObject.Update ();
		EditorGUILayout.PropertyField (objectPrefProp, new GUIContent ("Object Preference"));
		EditorGUILayout.PropertyField (allowJumpingProp, new GUIContent ("Allow jumping"));
		serializedObject.ApplyModifiedProperties ();
		AnnexRObject globalParams = (AnnexRObject)serializedObject.targetObject;

		//globalParams.mprint (); //Debug
	}
}
