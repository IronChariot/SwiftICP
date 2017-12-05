//
//  ViewController.swift
//  Swift ICP
//
//  Created by Sam Brooks on 03/12/2017.
//  Copyright © 2017 Sam Brooks. All rights reserved.
//

import UIKit
import KDTree
import GLKit
import Accelerate
import SceneKit

class ViewController: UIViewController {
    
    var scnView: SCNView!
    var scnScene: SCNScene!
    var cameraNode: SCNNode!
    
    func setupView() {
        scnView = self.view as! SCNView
    }
    
    func setupScene() {
        scnScene = SCNScene()
        scnView.scene = scnScene
    }
    
    func setupCamera() {
        cameraNode = SCNNode()
        cameraNode.camera = SCNCamera()
        cameraNode.position = SCNVector3(x: 0, y: 0, z: 10)
        scnScene.rootNode.addChildNode(cameraNode)
        scnView.allowsCameraControl = true
    }
    
    func createPoint(_ point: GLKVector3, color: UIColor) {
        let pointBall = SCNSphere(radius: 0.1)
        pointBall.firstMaterial?.diffuse.contents = color
        pointBall.firstMaterial?.specular.contents = UIColor.white
        let pointNode = SCNNode(geometry: pointBall)
        pointNode.name = "Point"
        pointNode.position = SCNVector3(point.x, point.y, point.z)
        scnScene.rootNode.addChildNode(pointNode)
    }

    override func viewDidLoad() {
        super.viewDidLoad()
        
        setupView()
        setupScene()
        setupCamera()
        
        let maxIter = 20
        
        // Create example point cloud (flat points)
        var points = [GLKVector3]()
        for i in 0...99 {
            for j in 0...99 {
                let x = (Float(i) / 10.0) - 3.0
                let y = (Float(j) / 10.0) - 7.0
                var z: Float = 0.0
                let sphereSurf = (9 - (x * x) - (y * y))
                if sphereSurf > 0 {
                    z = sphereSurf.squareRoot() - 1.5
                    if z < 0 { z = 0 }
                }
                let point = GLKVector3Make(Float(i)/10.0, Float(j)/10.0, z)
                points.append(point)
                createPoint(point, color: UIColor.cyan)
            }
        }
        
        var pointsOffset = [GLKVector3]()
        for i in 0...99 {
            for j in 0...99 {
                let x = (Float(i) / 10.0) - 5.0
                let y = (Float(j) / 10.0) - 5.0
                var z: Float = 0.0
                let sphereSurf = (9 - (x * x) - (y * y))
                if sphereSurf > 0 {
                    z = sphereSurf.squareRoot() - 1.5
                    if z < 0 { z = 0 }
                }
                let point = GLKVector3Make(Float(i)/10.0, Float(j)/10.0, z)
                pointsOffset.append(point)
            }
        }
        
        // Create a transformed point cloud (some degrees rotated around X axis,
        // translated by some units in x, y and z directions respectively)
        let xRot = (0 / 360) * 2 * Float.pi
        let xTrans: Float = 1.0
        let yTrans: Float = 1.0
        let zTrans: Float = 0.5
        
        var transformMatrix = GLKMatrix4Make(1.0, 0.0, 0.0, 0.0,
                                             0.0, 1.0, 0.0, 0.0,
                                             0.0, 0.0, 1.0, 0.0,
                                             0.0, 0.0, 0.0, 1.0)
        // Translation
        transformMatrix.m30 = xTrans
        transformMatrix.m31 = yTrans
        transformMatrix.m32 = zTrans
        
        // Rotation around x-Axis
        transformMatrix.m11 = cos(xRot)
        transformMatrix.m12 = sin(xRot)
        transformMatrix.m21 = -sin(xRot)
        transformMatrix.m22 = cos(xRot)
        
        var points2 = [GLKVector3]()
        for point in pointsOffset {
            let newVector = GLKMatrix4MultiplyVector4(transformMatrix, GLKVector4MakeWithVector3(point, 1))
            let newPoint = GLKVector3Make(newVector.x, newVector.y, newVector.z)
            points2.append(newPoint)
//            createPoint(newPoint, color: UIColor.red)
        }
        
        // Load first point cloud (base) into KDTree
        let tree: KDTree<GLKVector3> = KDTree(values: points)
        
        // First run: find minimum distance between points
        var minDist = 99999999.9
        for point in points2 {
            let closest = tree.nearest(toElement: point)
            let dist = closest?.squaredDistance(to: point).squareRoot()
            if dist! < minDist { minDist = dist! }
        }
        print(minDist)
        
        let ICPInstance = ICP()
        
        var error: Double = 9999.0
//        var lastError: Double = Double.greatestFiniteMagnitude
        for _ in 0..<maxIter {
            var trimmedPoints2 = [GLKVector3]()
            (trimmedPoints2, points, error) = ICPInstance.closestPoints(pointSet1Tree: tree, pointSet2: points2)
            print("Error: \(error)")
            points2 = ICPInstance.icpStep(pointSet1: points, pointSet2: trimmedPoints2, fullPointSet2: points2)
//            lastError = error
        }
        for point in points2 {
            createPoint(point, color: UIColor.green)
        }
    }
}

// Extend GLKVector3 to follow protocol required for use with KDTree
extension GLKVector3: KDTreePoint {
    public static var dimensions: Int {
        return 3
    }
    
    public static func ==(lhs: GLKVector3, rhs: GLKVector3) -> Bool {
        return GLKVector3AllEqualToVector3(lhs, rhs)
    }
    
    public func kdDimension(_ dimension: Int) -> Double {
        if dimension == 0 {
            return Double(self.x)
        } else if dimension == 1 {
            return Double(self.y)
        } else {
            return Double(self.z)
        }
    }
    
    public func squaredDistance(to otherPoint: _GLKVector3) -> Double {
        let distance = GLKVector3Distance(self, otherPoint)
        let distSq = distance * distance
        return Double(distSq)
    }
}

