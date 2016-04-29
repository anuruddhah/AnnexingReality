using UnityEngine;
using System.Collections;
using System;

namespace AnnexR{
	namespace Primitives{
		public enum Shape {Plane, Sphere, Cylinder, Cone, Torus, None};	
		
		public abstract class BaseParams : ScriptableObject{};
	}
}
