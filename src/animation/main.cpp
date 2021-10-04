// #include <iostream>
// #include <Particle.h>
// #include <ParticleSystem.h>
// #include <memory>



// void main(){
// //Initailize scene
// 	// One particle
// 	Particle p(0.0f, 10.0f, 0.0f);
// 	p.setLifetime(7.0f);
// 	//	p.setFixed(true);
// 	std::cout << "Lifetime =" << p.getLifetime() << std::endl;
// 	p.setBouncing(1.0f);
// 	p.addForce(0, -9.8f, 0);
// 	// One Plane
// 	Plane plane;
// 	plane.setPlaneNormal(0, 1, 0);
// 	plane.setPlanePoint(0, 0, 0);
// 	// Time values
// 	float dt = 0.01f;
// 	float tini = 0.0f;
// 	float tfinal = 10.0f;

// 	std::cout << "position = " << p.getCurrentPosition().x << "  " << p.getCurrentPosition().y
// 		<< "  " << p.getCurrentPosition().z << "  time = " << 0 << std::endl;
// 	int count = 0; //to count collisions

// 	// animation loop
// 	for (float t = tini+dt; t <= tfinal; t += dt){
// 		// call solver types: EulerOrig, EulerSemi and Verlet(to be implemented)
// 		p.updateParticle(dt, Particle::UpdateMethod::EulerSemi);
// 		std::cout << "position = " << p.getCurrentPosition().x << "  " << p.getCurrentPosition().y
// 			<< "  " << p.getCurrentPosition().z << "  time = " << t << std::endl;
// 		//Check Floor collisions
// 		if (p.collisionParticlePlane(plane)){  
// 			p.correctCollisionParticlePlain(plane);
// 			std::cout << "rebound = " << count++ << std::endl;
// 			system("PAUSE");
// 		}
// 	}
	
// 	system("PAUSE");
	
	
// }