/*
  This sample is not part of the C++ project. As such it is not included
  in the src/ folder. To use it, copy its contents into editor.p5js.org,
  and then run.
  
  What is this code's purpose? The first half of the Pikuma course uses
  P5JS to teach us the basics of vectors (by "us" I mean myself and the
  three voices in my head). While I am doing a parallel implementation
  in C++, I wanted to follow along as the course teaches.
*/

class vec2def {
  constructor(x, y) {
    this.x = x;
    this.y = y;
  }
  
  static add(a, b) {
    let result = new vec2def(0, 0);
    
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    
    return result;
  }
  
  add(v) {
    this.x += v.x;
    this.y += v.y;
  }
  
  static sub(a, b) {
    let result = new vec2def(0, 0);
    
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    
    return result;
  }
  
  sub(v) {
    this.x -= v.x;
    this.y -= v.y;
  }
  
  static scale(v, n) {
    let result = new vec2def(0, 0);
    
    result.x = v.x * n;
    result.y = v.y * n;
    
    return result;
  }
  
  scale(n) {
    this.x *= n;
    this.y *= n;
  }
  
  dot(v) {
    let x_prod = this.x * v.x;
    let y_prod = this.y * v.y;
    
    return x_prod + y_prod;
  }
  
  magnitude() {
    return Math.sqrt((this.x * this.x) + (this.y * this.y));
  }
  
  draw(color) {
    fill(color);
    stroke("white");
    circle(this.x, this.y, 10);
  }
}

let position = new vec2def(10, 20);
let velocity = new vec2def(2, 1);

function setup() {
  createCanvas(400, 400);
  
  position = vec2def.scale(position, 4);
}

function draw() {
  background("black");
  
  position.draw("red");
}
  in the src/ folder. To use it, copy its contents into editor.p5js.org,
  and then run.
  
  What is this code's purpose? The first half of the Pikuma course uses
  P5JS to teach us the basics of vectors (by "us" I mean myself and the
  three voices in my head). While I am doing a parallel implementation
  in C++, I wanted to follow along as the course teaches.
*/

class vec2def {
  constructor(x, y) {
    this.x = x;
    this.y = y;
  }
  
  static add(a, b) {
    let result = new vec2def(0, 0);
    
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    
    return result;
  }
  
  add(v) {
    this.x += v.x;
    this.y += v.y;
  }
  
  static sub(a, b) {
    let result = new vec2def(0, 0);
    
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    
    return result;
  }
  
  sub(v) {
    this.x -= v.x;
    this.y -= v.y;
  }
  
  static scale(v, n) {
    let result = new vec2def(0, 0);
    
    result.x = v.x * n;
    result.y = v.y * n;
    
    return result;
  }
  
  scale(n) {
    this.x *= n;
    this.y *= n;
  }
  
  magnitude() {
    return Math.sqrt((this.x * this.x) + (this.y * this.y));
  }
  
  draw(color) {
    fill(color);
    stroke("white");
    circle(this.x, this.y, 10);
  }
}

let position = new vec2def(10, 20);
let velocity = new vec2def(2, 1);

function setup() {
  createCanvas(400, 400);
  
  position = vec2def.scale(position, 4);
}

function draw() {
  background("black");
  
  position.draw("red");
}
  constructor(x, y) {
    this.x = x;
    this.y = y;
  }
  
  add(v) {
    this.x += v.x;
    this.y += v.y;
  }
  
  sub(v) {
    this.x -= v.x;
    this.y -= v.y;
  }
  
  scale(n) {
    this.x *= n;
    this.y *= n;
  }
  
  magnitude() {
    return Math.sqrt((this.x * this.x) + (this.y * this.y));
  }
  
  draw(color) {
    fill(color);
    stroke("white");
    circle(this.x, this.y, 10);
  }
}

let position = new vec2def(10, 20);
let velocity = new vec2def(2, 1);

function setup() {
  createCanvas(400, 400);
  position.scale(2.0);
}

function draw() {
  background("black");
  
  position.draw("red");
}
  in the src/ folder. To use it, copy its contents into editor.p5js.org,
  and then run.
  
  What is this code's purpose? The first half of the Pikuma course uses
  P5JS to teach us the basics of vectors (by "us" I mean myself and the
  three voices in my head). While I am doing a parallel implementation
  in C++, I wanted to follow along as the course teaches.
*/

class vec2def {
  constructor(x, y) {
    this.x = x;
    this.y = y;
  }
  
  add(v) {
    this.x += v.x;
    this.y += v.y;
  }
  
  sub(v) {
    this.x -= v.x;
    this.y -= v.y;
  }
  
  magnitude() {
    return Math.sqrt((this.x * this.x) + (this.y * this.y));
  }
  
  draw(color) {
    fill(color);
    stroke("white");
    circle(this.x, this.y, 10);
  }
}

let position = new vec2def(10, 20);
let velocity = new vec2def(2, 1);

function setup() {
  createCanvas(400, 400);
}

function draw() {
  background("black");
  
  position.add(velocity);
  
  position.draw("red");
}
