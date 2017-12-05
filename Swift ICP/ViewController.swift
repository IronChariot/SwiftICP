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
        print("Set2")
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
        
        // Create a transformed point cloud (45 degrees rotated around X axis,
        // translated by 10, 20 and 30 units in x, y and z directions respectively)
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
        
        // Rotation
        transformMatrix.m11 = cos(xRot)
        transformMatrix.m12 = sin(xRot)
        transformMatrix.m21 = -sin(xRot)
        transformMatrix.m22 = cos(xRot)
        
        print("\(transformMatrix.m00), \(transformMatrix.m10), \(transformMatrix.m20), \(transformMatrix.m30)")
        print("\(transformMatrix.m01), \(transformMatrix.m11), \(transformMatrix.m21), \(transformMatrix.m31)")
        print("\(transformMatrix.m02), \(transformMatrix.m12), \(transformMatrix.m22), \(transformMatrix.m32)")
        print("\(transformMatrix.m03), \(transformMatrix.m13), \(transformMatrix.m23), \(transformMatrix.m33)")
        
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
        
        var error: Double = 9999.0
        var lastError: Double = Double.greatestFiniteMagnitude
        for _ in 0..<maxIter {
            var trimmedPoints2 = [GLKVector3]()
            (trimmedPoints2, points, error) = closestPoints(pointSet1Tree: tree, pointSet2: points2)
            print("Error: \(error)")
            points2 = icp(pointSet1: points, pointSet2: trimmedPoints2, fullPointSet2: points2)
            lastError = error
        }
        for point in points2 {
            createPoint(point, color: UIColor.green)
        }
    }
    
    func closestPoints(pointSet1Tree: KDTree<GLKVector3>, pointSet2: [GLKVector3]) -> (trimmedSet2: [GLKVector3], pairedPoints: [GLKVector3], error: Double) {
        // For each point in second point cloud, find closest point in first cloud tree
        // TODO: Also consider something to find 'unique' pairs
        var pairedPoints = [GLKVector3]()
        var trimmedSet2 = [GLKVector3]()
        var totalError = 0.0
        for point in pointSet2 {
            let closest = pointSet1Tree.nearest(toElement: point)
            let xyDist = xyDistance(point1: closest!, point2: point)
            let distance = (closest?.squaredDistance(to: point))!.squareRoot()
//            if abs(distance - xyDist) > 0.0015 { // One and a half millimeter
                totalError += distance
                pairedPoints.append(closest!)
                trimmedSet2.append(point)
//            }
        }
        print("Length of trimmed set2: \(trimmedSet2.count)")
        let error = totalError / Double(pointSet2.count)
        
        return (trimmedSet2, pairedPoints, error)
    }
    
    func xyDistance(point1: GLKVector3, point2: GLKVector3) -> Double {
        let xD = (point1.x - point2.x)
        let yD = (point1.y - point2.y)
        
        return Double(((xD * xD) + (yD * yD)).squareRoot())
    }
    
    func icp(pointSet1: [GLKVector3], pointSet2: [GLKVector3], fullPointSet2: [GLKVector3]) -> [GLKVector3] {
        // Takes a fixed point cloud in a KDTree (pointSet1Tree) and a transformed point cloud pointSet2,
        // finds transform to move pointSet2 closer to pointSet1, returns thusly transformed pointSet2
        
        // Now that we have a set of pairs of points, find best rotation/translation to match up pairs
        var R: [Double]
        var T: [Double]
        (R, T) = getRTMatrixSVD(A: pointSet2, B: pointSet1, rotMat: GLKMatrix3Identity, tranVec: GLKVector3Make(0.0, 0.0, 0.0))
        
        var guessedTransformMatrix = GLKMatrix4Make(1.0, 0.0, 0.0, 0.0,
                                                    0.0, 1.0, 0.0, 0.0,
                                                    0.0, 0.0, 1.0, 0.0,
                                                    0.0, 0.0, 0.0, 1.0)
        // Translation
        guessedTransformMatrix.m30 = Float(T[0])
        guessedTransformMatrix.m31 = Float(T[1])
        guessedTransformMatrix.m32 = Float(T[2])
        
        // Rotation
        guessedTransformMatrix.m00 = Float(R[0])
        guessedTransformMatrix.m01 = Float(R[3])
        guessedTransformMatrix.m02 = Float(R[6])
        guessedTransformMatrix.m10 = Float(R[1])
        guessedTransformMatrix.m11 = Float(R[4])
        guessedTransformMatrix.m12 = Float(R[7])
        guessedTransformMatrix.m20 = Float(R[2])
        guessedTransformMatrix.m21 = Float(R[5])
        guessedTransformMatrix.m22 = Float(R[8])
        
        // Apply rotation/translation to second cloud (full), return
        var pointSet2Transformed = [GLKVector3]()
        for point in fullPointSet2 {
            let newVector = GLKMatrix4MultiplyVector4(guessedTransformMatrix, GLKVector4MakeWithVector3(point, 1))
            let newPoint = GLKVector3Make(newVector.x, newVector.y, newVector.z)
            pointSet2Transformed.append(newPoint)
        }
        
        return pointSet2Transformed
    }
    
    func getRTMatrixSVD(A: [GLKVector3], B: [GLKVector3], rotMat: GLKMatrix3, tranVec: GLKVector3) -> (R: [Double], T: [Double]) {
        // A is target point cloud, B is point cloud to be rotated/translated
        
        assert(A.count == B.count, "Point clouds to transform are not the same length")
        
        let N = A.count // Total points
        
        let centroidA = centroid(A)
        let centroidB = centroid(B)
        
        let H = covarianceMatrix(arrayA: A, arrayB: B, centroidA: centroidA, centroidB: centroidB)
        
        var u : [Double]
        var s : [Double]
        var v : [Double]
        (v, s, u) = svd(H, m: 3, n: 3)
        
        var R: [Double] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        // R = V * transpose(U)
        R[0] = v[0]*u[0] + v[3]*u[3] + v[6]*u[6]
        R[1] = v[0]*u[1] + v[3]*u[4] + v[6]*u[7]
        R[2] = v[0]*u[2] + v[3]*u[5] + v[6]*u[8]
        
        R[3] = v[1]*u[0] + v[4]*u[3] + v[7]*u[6]
        R[4] = v[1]*u[1] + v[4]*u[4] + v[7]*u[7]
        R[5] = v[1]*u[2] + v[4]*u[5] + v[7]*u[8]
        
        R[6] = v[2]*u[0] + v[5]*u[3] + v[8]*u[6]
        R[7] = v[2]*u[1] + v[5]*u[4] + v[8]*u[7]
        R[8] = v[2]*u[2] + v[5]*u[5] + v[8]*u[8]
        
        // TODO: Special reflection case needs to check if determinant is < 0
        
        var T: [Double] = [0.0, 0.0, 0.0]
        // T = -R*centroidA + transpose(centroidB)
        var bit1 = -R[0] * Double(centroidA.x)
        var bit2 = -R[1] * Double(centroidA.y)
        var bit3 = -R[2] * Double(centroidA.z)
        T[0] = bit1 + bit2 + bit3 + Double(centroidB.x)
        bit1 = -R[3] * Double(centroidA.x)
        bit2 = -R[4] * Double(centroidA.y)
        bit3 = -R[5] * Double(centroidA.z)
        T[1] = bit1 + bit2 + bit3 + Double(centroidB.y)
        bit1 = -R[6] * Double(centroidA.x)
        bit2 = -R[7] * Double(centroidA.y)
        bit3 = -R[8] * Double(centroidA.z)
        T[2] = bit1 + bit2 + bit3 + Double(centroidB.z)
        
        return (R, T)
    }
    
    func centroid(_ pointCloud: [GLKVector3]) ->  GLKVector3 {
        let vectorSum = pointCloud.reduce(GLKVector3Make(0, 0, 0), GLKVector3Add)
        let meanVector = GLKVector3DivideScalar(vectorSum, Float(pointCloud.count))
        return meanVector
    }
    
    func recenterArray(array: [GLKVector3], centre: GLKVector3) -> [GLKVector3] {
        var result = [GLKVector3]()
        
        for point in array {
            let newPoint = GLKVector3Subtract(point, centre)
            result.append(newPoint)
        }
        
        return result
    }
    
    func covarianceMatrix(arrayA: [GLKVector3], arrayB: [GLKVector3], centroidA: GLKVector3, centroidB: GLKVector3) -> [Double] {
        var H: [Double] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        for i in 0..<arrayA.count {
            let AX = (arrayA[i].x - centroidA.x)
            let AY = (arrayA[i].y - centroidA.y)
            let AZ = (arrayA[i].z - centroidA.z)
            let BX = (arrayB[i].x - centroidB.x)
            let BY = (arrayB[i].y - centroidB.y)
            let BZ = (arrayB[i].z - centroidB.z)
            
            H[0] += Double(AX*BX)
            H[1] += Double(AX*BY)
            H[2] += Double(AX*BZ)
            
            H[3] += Double(AY*BX)
            H[4] += Double(AY*BY)
            H[5] += Double(AY*BZ)
            
            H[6] += Double(AZ*BX)
            H[7] += Double(AZ*BY)
            H[8] += Double(AZ*BZ)
        }
        
        return H
    }
    
    func svd(_ x:[Double], m:Int, n:Int) -> (u:[Double], s:[Double], v:[Double]) {
        var JOBZ = Int8(UnicodeScalar("A").value)
        var M = __CLPK_integer(m)
        var N = __CLPK_integer(n)
        var A = x
        var LDA = __CLPK_integer(m)
        var S = [__CLPK_doublereal](repeating: 0.0, count: min(m,n))
        var U = [__CLPK_doublereal](repeating: 0.0, count: m*m)
        var LDU = __CLPK_integer(m)
        var VT = [__CLPK_doublereal](repeating: 0.0, count: n*n)
        var LDVT = __CLPK_integer(n)
        let lwork = min(m,n)*(6+4*min(m,n))+max(m,n)
        var WORK = [__CLPK_doublereal](repeating: 0.0, count: lwork)
        var LWORK = __CLPK_integer(lwork)
        var IWORK = [__CLPK_integer](repeating: 0, count: 8*min(m,n))
        var INFO = __CLPK_integer(0)
        dgesdd_(&JOBZ, &M, &N, &A, &LDA, &S, &U, &LDU, &VT, &LDVT, &WORK, &LWORK, &IWORK, &INFO)
        var s = [Double](repeating: 0.0, count: m*n)
        for ni in 0...n-1 {
            s[ni*m+ni] = S[ni]
        }
        var v = [Double](repeating: 0.0, count: n*n)
        vDSP_mtransD(VT, 1, &v, 1, vDSP_Length(n), vDSP_Length(n))
        return (U, s, v)
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }


}

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

