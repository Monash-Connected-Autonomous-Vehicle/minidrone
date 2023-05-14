import React, { Suspense } from 'react';
import { Canvas, useFrame } from 'react-three-fiber';
import { OrbitControls, useGLTF, Environment } from "@react-three/drei";

function Model(props) {
    const { scene } = useGLTF("/assembly_v1.glb");

    // useFrame(() => {
    //     sce
    // })

    return <primitive object = {scene} />;
}

export default function Scene() {
    return (
        <Canvas shadows pixelratio={[1, 2]} camera={{ position: [-2,1,1], fov: 50 }}>
            <ambientLight intensity={1} />
            <Suspense fallback={null}>
                <Model />
            </Suspense>
            <OrbitControls autoRotate />
        </Canvas>
    )
}