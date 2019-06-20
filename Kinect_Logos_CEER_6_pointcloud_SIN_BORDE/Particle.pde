// The Nature of Code
// <http://www.shiffman.net/teaching/nature>
// Spring 2010
// Box2DProcessing example

// A circular particle

class Particle {
  boolean descubierto; 
  PImage img_empresa;
  PImage img_incognita;
  // We need to keep track of a Body and a radius
  Body body;
  float r;
  color col;
  // Define a body
  BodyDef bd = new BodyDef();
  FixtureDef fd = new FixtureDef();
  boolean mostrar_grande;
  int touch;
  
  Particle(Vec2 coord) {
    descubierto = false;
    mostrar_grande = false;
    touch = 0;
    r = 38;
  
    img_incognita= loadImage("incognita.png");
    int marca = int(random(99))+1;
    img_empresa = loadImage(marca+".png");
    
    
    // Set its position
    bd.position = box2d.coordPixelsToWorld(coord.x,coord.y);
    bd.type = BodyType.DYNAMIC;
    // Make the body's shape a circle
    CircleShape cs = new CircleShape();
    cs.m_p.set(0,0);
    cs.m_radius = box2d.scalarPixelsToWorld(r);
    fd.shape = cs;
    // Parameters that affect physics
    fd.density = .2;
    fd.friction = 0.5;
    fd.restitution = 0.4;
    body = box2d.world.createBody(bd);
    // Attach fixture to body
    body.createFixture(fd);
    body.setLinearVelocity(new Vec2(random(-5, 5), random(2, 5)));
    col = color(175);
  }

  // This function removes the particle from the box2d world
  void killBody() {
    box2d.destroyBody(body);
  }
  
  // Is the particle ready for deletion?
  boolean done() {
    // Let's find the screen position of the particle
    Vec2 pos = box2d.getBodyPixelCoord(body);
    // Is it off the bottom of the screen?
    if (pos.y > height+r*2) {
      killBody();
      return true;
    }
    return false;
  }
  
  void estado_descubierto( boolean state) 
  {
  descubierto = state;
  mostrar_grande = true;
  fd.density = .8;
  }
  
  Vec2 ubica(){
    return box2d.getBodyPixelCoord(body);
  }
  
  void display() {
    // We look at each body and get its screen position
    Vec2 pos = ubica();
    // Get its angle of rotation
   // float a = body.getAngle();
    pushMatrix();
    translate(pos.x,pos.y);
    //rotate(a);
    fill(col);
    noStroke();
    //strokeWeight(1);
    if (mostrar_grande)
    {
       tocado();
    }
     if (descubierto)
     { 
    ellipse(0,0,r*2,r*2);
    // Let's add a line so we can see the rotation
    //line(0,0,r,0);
    pushMatrix();
    translate(-40,-39);
    scale(0.27,0.27);
    imageMode(CORNER);
    image(img_empresa, 0, 0);
    popMatrix(); 
   }
    else
    {
      ellipse(0,0,r*2,r*2);
      pushMatrix();
      translate(-40,-39);
      scale(0.45,0.45);
      imageMode(CORNER);
      image(img_incognita, 0,0);
      popMatrix(); 
  
   }
    popMatrix();
 }
 void attract(Vec2 adonde) {
    // From BoxWrap2D example
    Vec2 worldTarget = box2d.coordPixelsToWorld(adonde.x,adonde.y);   
    Vec2 bodyVec = body.getWorldCenter();
    // First find the vector going from this body to the specified point
    worldTarget.subLocal(bodyVec);
    // Then, scale the vector to the specified force
    worldTarget.normalize();
    if(!descubierto)
    {
    worldTarget.mulLocal((float) 100);
    }
    else
    {
    worldTarget.mulLocal((float) 1);
    }
    // Now apply it to the body's center of mass.
    body.applyForce(worldTarget, bodyVec);
  }

void tocado()
  {
      pushMatrix();
      translate(-400,-200);
      scale(3,3);
      imageMode(CORNER);
      image(img_empresa, 0, 0);
      popMatrix(); 
      if(frameCount%2 ==1)
      {
        touch++;
      }
      if(touch > 32)
      {
        mostrar_grande = false;
      }
      
  }
}



  
