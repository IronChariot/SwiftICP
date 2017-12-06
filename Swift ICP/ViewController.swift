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
    
    let maxIter = 20
    let minErrorChange = 0.0001
    
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
    
    func createPoint(_ point: GLKVector3, _ color: UIColor, showFraction: Double, size: Double) {
        // Make only a fraction of points show up... Stops SceneKit from getting too slow
        assert(showFraction <= 1.0, "Fractions shouldn't go above 1!")
        let willShow = drand48()
        if willShow > showFraction {
            return
        }
        
        // Actually putting a point in the scene
        let pointBall = SCNSphere(radius: CGFloat(size))
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
        
        simpleExample()
//        fileExample()
    }
    
    func simpleExample() {
        // Create example point cloud (flat points)
        var points = [GLKVector3]()
        for i in 0...99 {
            for j in 0...99 {
                let x = (Float(i) / 10.0) - 6.0
                let y = (Float(j) / 10.0) - 6.0
                var z: Float = 0.0
                let sphereSurf = (9 - (x * x) - (y * y))
                if sphereSurf > 0 {
                    z = sphereSurf.squareRoot() - 1.5
                    if z < 0 { z = 0 }
                }
                // Add a bit of noise to Z, between -0.1 and 0.1
                let noise = ((Float(arc4random_uniform(10000)) / 10000) * 0.2) - 0.1
                let point = GLKVector3Make(Float(i)/10.0, Float(j)/10.0, z + noise)
                points.append(point)
                createPoint(point, UIColor.cyan, showFraction: 0.1, size: 0.05)
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
                // Add a bit of noise to Z, between -0.1 and 0.1
                let noise = ((Float(arc4random_uniform(10000)) / 10000) * 0.2) - 0.1
                let point = GLKVector3Make(Float(i)/10.0, Float(j)/10.0, z + noise)
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
        }
        
        let ICPInstance = ICP(points, points2)
        let finalTransform = ICPInstance.iterate(maxIterations: 100, minErrorChange: 0.0)
        
        var finalPoints2 = [GLKVector3]()
        for point in points2 {
            createPoint(point, .red, showFraction: 0.1, size: 0.03)
            let newVector = GLKMatrix4MultiplyVector4(finalTransform, GLKVector4MakeWithVector3(point, 1))
            let newPoint = GLKVector3Make(newVector.x, newVector.y, newVector.z)
            finalPoints2.append(newPoint)
            createPoint(newPoint, UIColor.green, showFraction: 0.1, size: 0.05)
        }
    }
    
    func fileExample() {
        let points = getPointCloudFromFile(fileName: "realPointCloud0.csv", subsample: 30000)
        let points2 = getPointCloudFromFile(fileName: "realPointCloud1.csv", subsample: 30000)
        
        let ICPInstance = ICP(points, points2)
        let finalTransform = ICPInstance.iterate(maxIterations: maxIter, minErrorChange: 0.0)
        
        for point in points {
            createPoint(point, UIColor.blue, showFraction: 0.1, size: 0.003)
        }
        var finalPoints2 = [GLKVector3]()
        for point in points2 {
            createPoint(point, UIColor.red, showFraction: 0.1, size: 0.002)
            let newVector = GLKMatrix4MultiplyVector4(finalTransform, GLKVector4MakeWithVector3(point, 1))
            let newPoint = GLKVector3Make(newVector.x, newVector.y, newVector.z)
            finalPoints2.append(newPoint)
            createPoint(newPoint, UIColor.green, showFraction: 0.1, size: 0.003)
        }
    }
    
    func getPointCloudFromFile(fileName: String, subsample: Int) -> [GLKVector3] {
        // Extract an x, y, z point cloud from file, and subsample a maximum of 'subsample' points
        
        var pointCloud = [GLKVector3]()
        
        if let dir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first {
            let fileUrl = dir.appendingPathComponent(fileName)
            var text = ""
            
            do {
                text = try String(contentsOf: fileUrl, encoding: .utf8)
            } catch {
                print("Could not read from file '\(fileName)'")
            }
            
            var lines = text.components(separatedBy: .newlines)
            if lines[0].hasPrefix("x,") {
                lines.remove(at: 0)
            }
            
            let numberOfPoints = lines.count
            var fraction = 1.0
            if numberOfPoints > subsample {
                fraction = Double(subsample) / Double(numberOfPoints)
            }
            
            for line in lines {
                let willInclude = drand48()
                if willInclude > fraction {
                    continue // Skip to next line
                }
                let coords = line.components(separatedBy: ",")
                if coords.count < 3 {
                    // End of file
                    break
                }
                let x = Float(coords[0])
                let y = Float(coords[1])
                let z = Float(coords[2])
                let vec = GLKVector3Make(x!, y!, z!)
                pointCloud.append(vec)
            }
        }
        
        return pointCloud
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

