window.addEventListener('load', init);

// Window size
const width = 960;
const height = 540;

var wr = 0; // right wheel angle
var wl = 0; // left wheel angle
var L = 1.0; // wheel base
var dt = 0.1; // integration time
var ofz = 0.5; // height offset
var we = 0.1; // error (standard deviation) of wheel angle measurement
var Sigma = new THREE.Matrix4; // Error covariance matrix

var c = new THREE.Vector4(0,0,ofz,0); // center of rover (x,y,z,theta)

var c0 = new THREE.Vector3(); // center of body
var c1 = new THREE.Vector3(); // center of arm1
var c2 = new THREE.Vector3(); // center of arm2
var c3 = new THREE.Vector3(); // center of arm3
var q01 = new THREE.Quaternion(); // 0A1 matrix
var q02 = new THREE.Quaternion(); // 0A2 matrix
var q03 = new THREE.Quaternion(); // 0A3 matrix
var p0 = new THREE.Vector3(); // position of base origin
var p1 = new THREE.Vector3(); // position of joint 1
var p2 = new THREE.Vector3(); // position of joint 2
var p3 = new THREE.Vector3(); // position of joint 3
var pe = new THREE.Vector3(); // position of hand
var ve = new THREE.Vector3(); // velocity of hand
var l = [1, 3, 4, 4]; // arm length
var phi = [0, 45, 90];
var jacobi = new THREE.Matrix3(); // Jacobian
var i =0;

var dt = 0.1;
var start = [2,0.5,3];
var	end = [-3,-2.9,1];
var move = 0;
var T = 5;
var t0 = 0;



// For DK and IK
var v = 0; // actual liner velocity of rover
var w = 0; // actual angular velocity of rover
var vr = 0; // right wheel velocity
var vl = 0; // left wheel velocity
var vel = 0; // desired linear velocity
var omg = 0; // desired angular velocity

// For feedback control
var t = new THREE.Vector4(0,0,0,0); // center of target (x,y,z,theta)
var velt = 0; // target linear velocity
var omgt = 0; // target angular velocity
var feedback = 0; // feedback is on (1) or off (0)

// Map
var N = 10;
var wd = 1;
var obstacles = [[-5,2],[1,0],[0,2],[4,3],[2,-2],[2,-3],[2,-2],[2,0],[2,1],[2,2],[1,-4],[0,3],[0,-5],[-1,-5],[-1,-4],[-1,-2],[-1,2],[-2,-5],[-2,-4],[-2,2],[-3,-2],[-3,-1],[-3,0],[-3,3],[-4,2],[-5,-4],[-5,-3],[-5,-2],[1,2]];

var map;

// Dijkstra method
var start_pos = [-5, 3, 180];
var goal_pos = [-5, -5, 180];
//var goal_pos = [1, -5, 180];
let path = [];

// Follow trajectory
var target_number = 0;
var iteration = 0;

function init() {
  // renderer 
  const renderer = new THREE.WebGLRenderer({
    canvas: document.querySelector('#myCanvas')
  });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(width, height);
  renderer.setClearColor(0xffffff);

  // scene
  const scene = new THREE.Scene();

  // camera
  const camera = new THREE.PerspectiveCamera(45, width / height, 1, 10000);
  camera.up.x = 0; camera.up.y = 0; camera.up.z = 1;    
  camera.position.set(7, 7, 7);

  // camera controller
  const controls = new THREE.OrbitControls(camera);
  controls.enableDamping = true;
  controls.dampingFactor = 0.2;

  // parallel light
  const directionalLight = new THREE.DirectionalLight( 0xFFFFFF, 0.7 );
  directionalLight.position.set(0, 1, 1);
  scene.add( directionalLight );

  // ambient light
  const ambientLight = new THREE.AmbientLight( 0xf0f0f0 ); // soft white light
  scene.add(ambientLight);

  Sigma.set(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
  c.set(start_pos[0], start_pos[1], ofz, start_pos[2]*Math.PI/180.0);
  t.set(start_pos[0], start_pos[1], 0, start_pos[2]*Math.PI/180.0);

  // floor mesh
  //createFloor();

  function createFloor() {
    var geometry = new THREE.Geometry();
    for( var i=0; i<N; i++){
      for( var j=0; j<=N; j++){
        geometry.vertices.push( new THREE.Vector3((i - N/2 ) * wd, (j - N/2 ) * wd, 0) );
        geometry.vertices.push( new THREE.Vector3(((i+1) - N/2 ) * wd, (j - N/2 ) * wd, 0) );
      }
      for( var j=0; j<=N; j++){
        geometry.vertices.push( new THREE.Vector3((j - N/2 ) * wd, (i - N/2 ) * wd, 0) );
        geometry.vertices.push( new THREE.Vector3((j - N/2 ) * wd, ((i+1) - N/2 ) * wd, 0) );
      }
    }
    var material = new THREE.LineBasicMaterial({ color: 0xFFFFFF, transparent:true, opacity:0.5 });
    lines = new THREE.LineSegments(geometry, material);
    scene.add(lines);
  }

// base
  createBase();

  function createBase() {
    var geometry_base = new THREE.BoxGeometry(1, 1, l[0]);
    var material_base = new THREE.MeshStandardMaterial({color: 0xff0000, side: THREE.DoubleSide});
    base = new THREE.Mesh(geometry_base, material_base);
    scene.add(base);
  }

  // arm 1
  createArm1();

  function createArm1() {
    var geometry = new THREE.BoxGeometry(0.2, 0.2, l[1]);
    var material = new THREE.MeshStandardMaterial({color: 0x00ff00, side: THREE.DoubleSide});
    arm1 = new THREE.Mesh(geometry, material);
    scene.add(arm1);
  }

  // arm 2
  createArm2();

  function createArm2() {
    var geometry = new THREE.BoxGeometry(l[2], 0.2, 0.2);
    var material = new THREE.MeshStandardMaterial({color: 0x0000ff, side: THREE.DoubleSide});
    arm2 = new THREE.Mesh(geometry, material);
    scene.add(arm2);
  }

  // arm 3
  createArm3();

  function createArm3() {
    var geometry = new THREE.BoxGeometry(l[3], 0.2, 0.2);
    var material = new THREE.MeshStandardMaterial({color: 0xff00ff, side: THREE.DoubleSide});
    arm3 = new THREE.Mesh(geometry, material);
    scene.add(arm3);
  }

  DK_M();

  // set map
  createMap();

  function createMap() {
    map = new Array(N);
    for( var i=0; i<N; i++){
      map[i] = new Array(N);
      for( var j=0; j<=N; j++){
           map[i][j] = 0;
      }
    }

    for( var i=0; i<obstacles.length; i++){
      var x = obstacles[i][0] / wd + N / 2;
      var y = obstacles[i][1] / wd + N / 2;
      if (x > N - 1 ) x = N - 1;
      if (y > N - 1 ) y = N - 1;
      map[x][y] = 1;
    }

    for( var i=0; i<N; i++){
      for( var j=0; j<N; j++){
        var material;
        if (map[i][j] != 0 ){
          material = new THREE.MeshBasicMaterial( { color: 0xee0000 } );
        } else {
          if ( (i + j) % 2 == 0 ) {
            material = new THREE.MeshBasicMaterial( { color: 0xe0e0e0 } );
          } else {
            material = new THREE.MeshBasicMaterial( { color: 0xeeeeee } );
          }
        }
        var geometry = new THREE.CircleGeometry( wd / Math.sqrt(2), 4, Math.PI / 4 );
        var mesh = new THREE.Mesh( geometry, material );
        mesh.position.set((i - N/2 ) * wd, (j - N/2 ) * wd, 0);
        scene.add(mesh);
      }
    }
  }

  // rover
  createRover();

  function createRover() {

    var base = new THREE.Mesh(
        new THREE.BoxGeometry(1.4, 1, 1),
        new THREE.MeshLambertMaterial({color: 0xff0000, side: THREE.DoubleSide})
    );

    var wheel0 = new THREE.Mesh(
        new THREE.CircleGeometry(0.2),
        new THREE.MeshStandardMaterial({color: 0x00ff00, side: THREE.DoubleSide})
    );

    var wheel1 = new THREE.Mesh(
        new THREE.CircleGeometry(0.5),
        new THREE.MeshStandardMaterial({color: 0x00ff00, side: THREE.DoubleSide})
    );

    var wheel2 = new THREE.Mesh(
        new THREE.CircleGeometry(0.5),
        new THREE.MeshStandardMaterial({color: 0x00ff00, side: THREE.DoubleSide})
    );

    rover = new THREE.Group();

    rover.add(base);
    base.position.set(0.2, 0, 0.2);

    rover.add(wheel0);
    wheel0.position.set(0.7, 0, -0.3);
    wheel0.rotation.set(Math.PI/2,0,0);

    rover.add(wheel1);
    wheel1.position.set(0, 0.55, 0);
    wheel1.rotation.set(Math.PI/2,0,0);

    rover.add(wheel2);
    wheel2.position.set(0, -0.55, 0);
    wheel2.rotation.set(Math.PI/2,0,0);

    scene.add(rover);
  }

  // target
  createTarget();

  function createTarget() {
   var shape = new THREE.Shape();
   shape.moveTo(  1, 0 );
   shape.lineTo( -1, 0.5, );
   shape.lineTo( -1,-0.5 );
   shape.lineTo(  1, 0 );
   var geometry = new THREE.ShapeGeometry( shape );
   var material = new THREE.MeshBasicMaterial( { color: 0xeeee00 } );
   target = new THREE.Mesh( geometry, material );
   scene.add( target );
  }

  createEllipse();

  function createEllipse() {
    var geometry = new THREE.Geometry();
    var N = 10;
    var r = 1;
    for( var i=0; i<N; i++){
      geometry.vertices.push( new THREE.Vector3(r *  Math.cos(i / N * 2.0 * Math.PI), r *  Math.sin(i / N * 2.0 * Math.PI), 0) );
      geometry.vertices.push( new THREE.Vector3(r *  Math.cos((i+1) / N * 2.0 * Math.PI), r *  Math.sin((i+1) / N * 2.0 * Math.PI), 0) );
    }
    var material = new THREE.LineBasicMaterial({ color: 0xFFFFFF, transparent:true, opacity:0.5 });
    ellipse = new THREE.LineSegments(geometry, material);
    scene.add(ellipse);
  }

  Dijkstra();

  function Dijkstra() {
    //Insert code for Dijkstra method here
    //from goal position (goal_pos)  to start position (start_pos)

    var Q;                // ダイクストラ法で使う頂点リスト
    var ux, uy;           // ダイクストラ法で使う位置
    var d;                // ダイクストラ法で使う疑似距離
    var prx, pry;         // ダイクストラ法で使う一つ前の位置
    var mind;             // ダイクストラ法で使う最小のd

    Q = new Array(N);
    d = new Array(N);
    prx = new Array(N);
    pry = new Array(N);
    for( var i=0; i<N; i++){
      Q[i] = new Array(N);
      d[i] = new Array(N);
      prx[i] = new Array(N);
      pry[i] = new Array(N);
    }

    // 初期位置のグリッド番号
    var sx = start_pos[0] / wd + N / 2;
    var sy = start_pos[1] / wd + N / 2;

    sx = Math.max(sx, 0);
    sx = Math.min(sx, N-1);
    sy = Math.max(sy, 0);
    sy = Math.min(sy, N-1);

    // 初期化
    for (var i = 0; i < N; i++) {
        for (var j = 0; j < N; j++) {
            if (i == sx && j == sy) {
                d[i][j] = 0;
            } 
            else {
                d[i][j] = 9999; // 大きな値（9999）を設定
            }
        }
    }

    for (var i = 0; i < N; i++) {
        for (var j = 0; j < N; j++) {
            prx[i][j] = 0;
            pry[i][j] = 0;
        }
    }

    for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
            if (map[i][j] == 0) { // 障害物ではない
                Q[i][j] = 1;
            }
            else { // 障害物(map[i][j] が 1)なら、頂点リストに入れない
                Q[i][j] = 0;
            }
        }
    }

    // 目的位置のグリッド番号
    var gx = goal_pos[0] / wd + N / 2;
    var gy = goal_pos[1] / wd + N / 2;

    gx = Math.max(gx, 0);
    gx = Math.min(gx, N-1);
    gy = Math.max(gy, 0);
    gy = Math.min(gy, N-1);

    while (1) {
        // 計算していない点（＝頂点リストQに残っている点）があるかチェック
        var flg = 0;
        for (var i = 0; i < N; i++)
            for (var j = 0; j < N; j++)
                if (Q[i][j] != 0) flg = 1;
        if (flg == 0) break; // 計算していない点がないなら終了

        // 計算していない点（＝頂点リストQに残っている点）から、dが最小の点(ux,uy)を探す
        mind = 9999;
        for (var i = 0; i < N; i++) {
            for (var j = 0; j < N; j++) {
                if (Q[i][j] == 1) {
                    if (mind > d[i][j]) {
                        ux = i;
                        uy = j;
                        mind = d[i][j];
                    }
                }
            }
        }

        // 頂点リストQから(ux,uy)を取り除く
        Q[ux][uy] = 0;

        // (ux,uy)と繋がるすべての頂点で、dを再計算する
        var nx, ny;
        var cost = 1; // 移動コストは1に設定

        nx = ux - 1;
        ny = uy;
        if (nx >= 0 && nx < N && ny >= 0 && ny < N) {
            if (map[nx][ny] == 0) {
                if (d[nx][ny] > d[ux][uy] + cost) {
                    d[nx][ny] = d[ux][uy] + cost;
                    prx[nx][ny] = ux;
                    pry[nx][ny] = uy;
                }
            }
        }

        nx = ux + 1;
        ny = uy;
        if (nx >= 0 && nx < N && ny >= 0 && ny < N) {
            if (map[nx][ny] == 0) {
                if (d[nx][ny] > d[ux][uy] + cost) {
                    d[nx][ny] = d[ux][uy] + cost;
                    prx[nx][ny] = ux;
                    pry[nx][ny] = uy;
                }
            }
        }

        nx = ux;
        ny = uy - 1;
        if (nx >= 0 && nx < N && ny >= 0 && ny < N) {
            if (map[nx][ny] == 0) {
                if (d[nx][ny] > d[ux][uy] + cost) {
                    d[nx][ny] = d[ux][uy] + cost;
                    prx[nx][ny] = ux;
                    pry[nx][ny] = uy;
                }
            }
        }

        nx = ux;
        ny = uy + 1;
        if (nx >= 0 && nx < N && ny >= 0 && ny < N) {
            if (map[nx][ny] == 0) {
                if (d[nx][ny] > d[ux][uy] + cost) {
                    d[nx][ny] = d[ux][uy] + cost;
                    prx[nx][ny] = ux;
                    pry[nx][ny] = uy;
                }
            }
        }
    }

    // Goalから順にたどる
    var si = gx;
    var sj = gy;
    while(1) {
        const p = { x: (si - N/2 ) * wd, y: (sj - N/2 ) * wd };
        path.push(p);
        if (si == sx && sj == sy) break;
        var ni = prx[si][sj];
        var nj = pry[si][sj];
        si = ni;
        sj = nj;
    }

    target_number =  path.length - 1;

  }

  // set path
  createPath();

  function createPath() {
    for( var n=0; n<path.length; n++){
      var material = new THREE.MeshBasicMaterial( { color: 0x00ee00 } );
      var geometry = new THREE.CircleGeometry( wd / Math.sqrt(2), 4, Math.PI / 4 );
      var mesh = new THREE.Mesh( geometry, material );
      mesh.position.set(path[n].x, path[n].y, 0.0);
      scene.add(mesh);
    }
  }

  // rendering
  tick();

  // keyboad control
  var PosSpeed = 0.1;
  var RotSpeed = 0.01;

  document.addEventListener("keydown", onDocumentKeyDown, false);
  function onDocumentKeyDown(event) {
    var keyCode = event.which;

    if (keyCode == 90) {
      // z
      velt -= PosSpeed;
    } else if (keyCode == 65) {
      // a
      omgt += RotSpeed;
    } else if (keyCode == 83) {
      // s
      omgt -= RotSpeed;
    } else if (keyCode == 87) {
      // w
      velt += PosSpeed;
    } else if (keyCode == 71) {
      // g
      if(feedback == 0) {
        feedback = 1;
      } else {
        feedback = 0;
        vel = 0;
        omg = 0;
      }
    } else if (keyCode == 32) {
      // space
      feedback = 0;
      vel = 0;
      omg = 0;
      c.set(start_pos[0], start_pos[1], ofz,  start_pos[2]*Math.PI/180.0);
      t.set(start_pos[0], start_pos[1], 0, start_pos[2]*Math.PI/180.0);
      Sigma.set(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
      velt = 0;
      omgt = 0;
      t.set(0, 0, 0, 0);
    } 
	  else if (keyCode==75){//k
		  start.copy(pe);
      ve.set(0,0,0);
      move = 0;
      console.log("Set start " + start.x + "  " + start.y + "  " + start.z)
    } else if (keyCode == 76) {
      // l
      end.copy(pe);
      ve.set(0,0,0);

      pe.copy(start);
      IK_M();
      DK_M();
      move = 1;
      t = 0;
      console.log("Set end " + end.x + "  " + end.y + "  " + end.z)
    } else if (keyCode == 32) {
      phi[0] =  0.0;
      phi[1] = 45.0;
      phi[2] = 90.0;
      ve.set(0,0,0);

      DK_M();
	  }
	
	  else {
      feedback = 0;
      vel = 0;
      omg = 0;
      velt = 0;
      omgt = 0;
    
	}
	move = 1;
	
	//start = [0.5,0.5,0.5];
	//end = [1,1,1];
	console.log("kek");
	
	//move = 1;
	//end.copy(pe)

	IK_M();
    DK_M();
    updateTarget();


    // from target position (t) and velocity (vt) to body velocity (vel, omg)
    if(feedback == 1) Feedback();

    // from body velocity (vel, omg) to wheel velocity (vr, vl)
    IK();

    // from wheel velocity (vr, vl) to body velocity (v ,w)
    DK();

    updatePosition();

  }
  
function IK_M() {

    function sqr(x) {
      return x*x;
    }

    phi[0] = Math.atan2(pe.y, pe.x);

    r = Math.sqrt(sqr(pe.x/Math.cos(phi[0])) + sqr(pe.z-l[0]-l[1]));
    p = Math.atan2(pe.z-l[0]-l[1], pe.x/Math.cos(phi[0]));
    d = 1.0/(2.0*l[2])*(sqr(pe.x/Math.cos(phi[0]))
         + sqr(pe.z-l[0]-l[1]) + sqr(l[2])-sqr(l[3]));

    if(d>r) {
        console.log("Out of region\n");
        return;
    }

    phi[1] = Math.atan2(d/r, Math.sqrt(1.0-sqr(d/r))) - p;
    phi[2] = Math.atan2(pe.x/Math.cos(phi[0])-Math.sin(phi[1])*l[2],pe.z-l[0]-l[1]-Math.cos(phi[1])*l[2])-phi[1];

    phi[0] = phi[0] * 180.0 / Math.PI; 
    phi[1] = phi[1] * 180.0 / Math.PI; 
    phi[2] = phi[2] * 180.0 / Math.PI; 

//    console.log("phi[0] " + phi[0]);
//    console.log("phi[1] " + phi[1]);
//    console.log("phi[2] " + phi[2]);

  }

  function calcJacobi() {
    var C1 = Math.cos(phi[0]*Math.PI/180.0);
    var S1 = Math.sin(phi[0]*Math.PI/180.0);
    var C2 = Math.cos(phi[1]*Math.PI/180.0);
    var S2 = Math.sin(phi[1]*Math.PI/180.0);
    var C3 = Math.cos(phi[2]*Math.PI/180.0);
    var S3 = Math.sin(phi[2]*Math.PI/180.0);
    var C23 = Math.cos((phi[1]+phi[2])*Math.PI/180.0);
    var S23 = Math.sin((phi[1]+phi[2])*Math.PI/180.0);

    jacobi.set( -S1*(S2*l[2]+S23*l[3]), C1*(C2*l[2]+C23*l[3]), C1*C23*l[3],
                 C1*(S2*l[2]+S23*l[3]), S1*(C2*l[2]+C23*l[3]), S1*C23*l[3],
                                    0,      -S2*l[2]-S23*l[3],   -S23*l[3]);
   }
  
function DK_M() {
    var l0 = new THREE.Vector3(0, 0, l[0]);
    var l1 = new THREE.Vector3(0, 0, l[1]);
    var l2 = new THREE.Vector3(l[2], 0, 0);
    var l3 = new THREE.Vector3(l[3], 0, 0);
    var h0 = new THREE.Vector3(0, 0, l[0]/2.0);
    var h1 = new THREE.Vector3(0, 0, l[1]/2.0);
    var h2 = new THREE.Vector3(l[2]/2.0, 0, 0);
    var h3 = new THREE.Vector3(l[3]/2.0, 0, 0);

    var x_axis = new THREE.Vector3(1, 0, 0);
    var y_axis = new THREE.Vector3(0, 1, 0);
    var z_axis = new THREE.Vector3(0, 0, 1);

    var q1 = new THREE.Quaternion().setFromAxisAngle(x_axis, -Math.PI/2.0);
    var q2 = new THREE.Quaternion().setFromAxisAngle(z_axis, -Math.PI/2.0);
    var q3 = new THREE.Quaternion().setFromAxisAngle(z_axis, phi[1]*Math.PI/180.0);
    var q12 = new THREE.Quaternion().multiplyQuaternions(q1,q2).multiply(q3);
    var q23 = new THREE.Quaternion().setFromAxisAngle(z_axis,phi[2]*Math.PI/180.0);

    q01.setFromAxisAngle(z_axis, phi[0]*Math.PI/180.0);
    q02 = q01.clone();
    q02.multiply(q12);
    q03 = q02.clone();
    q03.multiply(q23);

    p0.set(0, 0, 0);
    p1 = l0.clone();
    p2 = p1.clone();
    p2.add( l1.applyQuaternion( q01 ) );
    p3 = p2.clone();
    p3.add( l2.applyQuaternion( q02 ) );
    pe = p3.clone();
    pe.add( l3.applyQuaternion( q03 ) );

    c0 = h0.clone();
    c1 = p1.clone();
    c1.add( h1.applyQuaternion( q01 ) );
    c2 = p2.clone();
    c2.add( h2.applyQuaternion( q02 ) );
    c3 = p3.clone();
    c3.add( h3.applyQuaternion( q03 ) );

//    console.log(pe)
  }
  function DK() {
    //Insert code for direct kinematics (wheel odometry) here
    //from wheel velocity (vr, vl) to body velocity (v ,w)
    v = (vr + vl) / 2;
    w = (vr - vl) / L;
  }

  function IK() {
    //Insert code for inverse kinemamtics here
    //from body velocity (vel, omg) to wheel velocity (vr, vl)
    vr = (2 * vel + L * omg)/2;
    vl = (2 * vel - L * omg)/2;
  }

  function errorCalculation() {
    //Insert code for error calculation
    //Sigma = ...
    var A = new THREE.Matrix4();
    var B = new THREE.Matrix4();
    var W = new THREE.Matrix4();
    var At = new THREE.Matrix4();
    var Bt = new THREE.Matrix4();
    var m1 = new THREE.Matrix4();
    var m2 = new THREE.Matrix4();
    var m3 = new THREE.Matrix4();
    var m4 = new THREE.Matrix4();

    A.set(1, 0, -(vr + vl) / 2 * dt * Math.sin(c.w), 0,
          0, 1,  (vr + vl) / 2 * dt * Math.cos(c.w), 0,
          0, 0, 1, 0,
          0, 0, 0, 1);

    At = A.clone();
    At.transpose();

    B.set(dt * Math.cos(c.w) / 2, dt * Math.cos(c.w) / 2, 0, 0, 
          dt * Math.sin(c.w) / 2, dt * Math.sin(c.w) / 2, 0, 0, 
          dt / L, -dt / L, 0, 0, 
          0, 0, 0, 1);

    Bt = B.clone();
    Bt.transpose();

    W.set(we * we, 0, 0, 0,
          0, we * we, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0);

    m1.multiplyMatrices(A, Sigma); 
    m2.multiplyMatrices(m1, At); 

    var v2 = m2.elements;

    m3.multiplyMatrices(B, W); 
    m4.multiplyMatrices(m3, Bt); 

    var v4 = m4.elements;

    Sigma.set(v2[0]+v4[0], v2[4]+v4[4], v2[8]+v4[8], v2[12]+v4[12],
              v2[1]+v4[1], v2[5]+v4[5], v2[9]+v4[9], v2[13]+v4[13],
              v2[2]+v4[2], v2[6]+v4[6], v2[10]+v4[00], v2[14]+v4[14],
              v2[3]+v4[3], v2[7]+v4[7], v2[11]+v4[11], v2[15]+v4[15]);

    var s = Sigma.elements;

    //console.log("x, y, theta, xy " + s[0] + " " + s[5] + " " + s[10] + " " + s[1]);

  }

  function Feedback() {
    //Insert code for feedback control here
    //from target position (t) and velocity (velr, omgr) to body velocity (vel, omg)
    var Kx = 1;
    var Ky = 1;
    var Kth = 1;

    var phir = t.w;
    var omgr = omgt;
    var vr = velt;

    var ex =  (t.x-c.x) * Math.cos(c.w) + (t.y-c.y) * Math.sin(c.w);
    var ey = -(t.x-c.x) * Math.sin(c.w) + (t.y-c.y) * Math.cos(c.w);
    var ephi = phir - c.w;

    vel = vr * Math.cos(ephi) + Kx * ex;
    omg = omgr + vr * (Ky * ey + Kth * Math.sin(ephi));
  }

  function updatePosition() {
    var x = c.x + v * Math.cos(c.w) * dt;
    var y = c.y + v * Math.sin(c.w) * dt;
    var theta = c.w + w * dt;
    c.set(x, y, ofz, theta);
  }

  function updateTarget() {
    var x = t.x + velt * Math.cos(t.w) * dt;
    var y = t.y + velt * Math.sin(t.w) * dt;
    var theta = t.w + omgt * dt;
    t.set(x, y, 0, theta);
  }

  function tick() {
	  
    updateTarget();
	//manipulator part
	  pe.x = start[0] + (end[0] - start[0])/100 * i;
	  pe.y = start[1] + (end[1] - start[1])/100 * i;
	  pe.z = start[2] + (end[2] - start[2])/100 * i;
	  i = i+1;
	  IK_M();
	  DK_M();
	  if(i>100)
	  {
		  i=0;
		  start = end;

		  end = [c.x,c.y,c.z];
		  console.log(end);
	  }
	 calcJacobi();

    var phid = new THREE.Vector3();
    var ved  = new THREE.Vector3();
    ved.set(ve.x, ve.y, ve.z);

    var im = new THREE.Matrix3();

    var det = jacobi.determinant ();
    if(Math.abs(det) < 0.1) {
        console.log("Out of region\n");
        ve.set(0,0,0);
    }

    //console.log("determine " + det);

    im.getInverse(jacobi)

    phid = ved.applyMatrix3(im);
    
    phi[0] = phi[0] + phid.x / Math.PI * 180.0 * dt;
    phi[1] = phi[1] + phid.y / Math.PI * 180.0 * dt;
    phi[2] = phi[2] + phid.z / Math.PI * 180.0 * dt;

    DK_M();

    base.position.copy(c0);
    arm1.position.copy(c1);
    arm1.quaternion.copy(q01);
    arm2.position.copy(c2);
    arm2.quaternion.copy(q02);
    arm3.position.copy(c3);
    arm3.quaternion.copy(q03);  
	//rover part
    if(feedback == 1) {
      // Position
      if (target_number > 0){
        t.x = path[target_number].x + (iteration % 100) / 100.0 * (path[target_number-1].x - path[target_number].x);
        t.y = path[target_number].y + (iteration % 100) / 100.0 * (path[target_number-1].y - path[target_number].y);
      }
      // Orientation
      if (target_number == path.length - 1){
        var pt = start_pos[2]*Math.PI/180.0;
        var tt = Math.atan2(path[target_number-1].y - path[target_number].y, path[target_number-1].x - path[target_number].x);
        t.w = pt + (iteration % 100) / 100.0 * (tt - pt);
        velt = 0.1;
        omgt = 0.1 * (tt - pt);
      } else if (target_number > 0 && target_number < path.length - 1){
        var pt = Math.atan2(path[target_number].y - path[target_number+1].y, path[target_number].x - path[target_number+1].x);
        var tt = Math.atan2(path[target_number-1].y - path[target_number].y, path[target_number-1].x - path[target_number].x);
        t.w = pt + (iteration % 100) / 100.0 * (tt - pt);
        velt = 0.1;
        omgt = 0.1 * (tt - pt);
      } else  if (target_number == 0){
        velt = 0.0;
        omgt = 0.0;
      }

      iteration ++; 
      if ( target_number > 0 && iteration % 100 == 0) target_number -= 1;

      Feedback();

    }

    IK();

    DK();

    updatePosition();

    errorCalculation();

    var z_axis = new THREE.Vector3(0, 0, 1);
    var q = new THREE.Quaternion().setFromAxisAngle(z_axis, c.w);

    rover.position.set(c.x,c.y,c.z);
    rover.quaternion.copy(q);

    var qt = new THREE.Quaternion().setFromAxisAngle(z_axis, t.w);
    target.position.set(t.x,t.y,0.1);
    target.quaternion.copy(qt);

    var s = Sigma.elements;
    var delta = s[0]*s[5]-s[1]*s[4];
    var r1 = 0;
    var r2 = 0;
    var th = 0;
    if(delta != 0) {
      r1 = ((s[0]+s[5])+Math.sqrt((s[0]+s[5])*(s[0]+s[5])-4.0*delta))/delta/2.0;
      r2 = ((s[0]+s[5])-Math.sqrt((s[0]+s[5])*(s[0]+s[5])-4.0*delta))/delta/2.0;
      th = Math.atan2(s[4], s[0]-r1*delta);
      r1 = 1.0/Math.sqrt(r1);
      r2 = 1.0/Math.sqrt(r2);
    };
    var qe = new THREE.Quaternion().setFromAxisAngle(z_axis, th);
    //console.log(s[0] + " " + s[5] + " " + s[1] + " " + s[4]);

    ellipse.position.set(c.x,c.y,0);
    ellipse.quaternion.copy(qe);
    ellipse.scale.set(r1,r2,0);

    // update camera controller
    controls.update();

    // rendering
    renderer.render(scene, camera);

   // console.log("phi " + phi[0] + " " + phi[1] + " " + phi[2]);
    requestAnimationFrame(tick);
  }


}