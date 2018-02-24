  function Vector(x, y)
  {
    this.x = x;
    this.y = y;

    this.equal = function(v)
    {
      return this.x == v.getX() && this.y == v.getY();
    }
    this.getX = function()
    {
      return this.x;
    }
    this.getY = function()
    {
      return this.y;
    }
    this.setX = function(x)
    {
      this.x = x;
    }
    this.setY = function(y)
    {
      this.y = y;
    }
    this.addX = function(x)
    {
      this.x += x;
    }
    this.addY = function(y)
    {
      this.y += y;
    }
    this.set = function(v)
    {
      this.x = v.getX();
      this.y = v.getY();
    }
    this.add = function(v)
    {
      this.x += v.getX();
      this.y += v.getY();
    }
    this.sub = function(v)
    {
      this.x -= v.getX();
      this.y -= v.getY();
    }
    this.dotProd = function(v)
    {
      return this.x * v.getX() + this.y * v.getY();
    }
    this.dist = function(v)
    {
      return Math.sqrt((this.x - v.getX()) * (this.x - v.getX()) + (this.y - v.getY()) * (this.y - v.getY()));
    }
    this.length = function()
    {
      return Math.sqrt(this.x * this.x + this.y * this.y);
    }
    this.scale = function(scaleFactor)
    {
      this.x *= scaleFactor;
      this.y *= scaleFactor;
    }
    this.normalize = function()
    {
      var l = this.length();
      this.x /= l;
      this.y /= l;
    }
    this.toString = function()
    {
      return " X: " + this.x + " Y: " + this.y;
    }
  }

  function Environment(x, y, w, h)
  {
    this.left = x;
    this.right = x + w;
    this.top = y;
    this.buttom = y + h;
    this.r = new Vector(0.0, 0.0);

    this.collision = function(curPos, prevPos)
    {
      var collide = false;
      var i;

      if(curPos.getX() < this.left)
      {
        curPos.setX(this.left);
        collide = true;
      }
      else if(curPos.getX() > this.right)
      {
        curPos.setX(this.right);
        collide = true;
      }
      if(curPos.getY() < this.top)
      {
        curPos.setY(this.top);
        collide = true;
      }
      else if(curPos.getY() > this.buttom)
      {
        curPos.setY(this.buttom);
        collide = true;
      }
      return collide;
    }
    this.draw = function(ctx, scaleFactor)
    {
      ctx.lineWidth = 2;
      ctx.strokeStyle = '#cccccc';
      ctx.beginPath();
      ctx.moveTo(this.left * scaleFactor, this.top * scaleFactor);
      ctx.lineTo(this.right * scaleFactor, this.top * scaleFactor);
      ctx.lineTo(this.right * scaleFactor, this.buttom * scaleFactor);
      ctx.lineTo(this.left * scaleFactor, this.buttom * scaleFactor);
      ctx.closePath();
      ctx.stroke();
    }
  }

  function PointMass(cx, cy, mass)
  {
    this.cur = new Vector(cx, cy);
    this.prev = new Vector(cx, cy);
    this.mass = mass;
    this.force = new Vector(0.0, 0.0);
    this.result = new Vector(0.0, 0.0);
    this.friction = 0.01;

    this.getXPos = function()
    {
      return this.cur.getX();
    }
    this.getYPos = function()
    {
      return this.cur.getY();
    }
    this.getPos = function()
    {
      return this.cur;
    }
    this.getXPrevPos = function()
    {
      return this.prev.getX();
    }
    this.getYPrevPos = function()
    {
      return this.prev.getY();
    }
    this.getPrevPos = function()
    {
      return this.prev;
    }
    this.addXPos = function(dx)
    {
      this.cur.addX(dx);
    }
    this.addYPos = function(dy)
    {
      this.cur.addY(dy);
    }
    this.setForce = function(force)
    {
      this.force.set(force);
    }
    this.addForce = function(force)
    {
      this.force.add(force);
    }
    this.getMass = function()
    {
      return this.mass;
    }
    this.setMass = function(mass)
    {
      this.mass = mass;
    }
    this.setPos = function(x, y)
    {
      this.cur.setX(x);
      this.cur.setY(y);
    }
    this.move = function(dt)
    {
      var t, a, c, dtdt;

      dtdt = dt * dt;

      a = this.force.getX() / this.mass;
      c = this.cur.getX();
      t = (2.0 - this.friction) * c - (1.0 - this.friction) * this.prev.getX() + a * dtdt;
      this.prev.setX(c);
      this.cur.setX(t);

      a = this.force.getY() / this.mass;
      c = this.cur.getY();
      t = (2.0 - this.friction) * c - (1.0 - this.friction) * this.prev.getY() + a * dtdt;
      this.prev.setY(c);
      this.cur.setY(t);
    }
    this.setFriction = function(friction)
    {
      this.friction = friction;
    }
    this.getVelocity = function()
    {
      var cXpX, cYpY;

      cXpX = this.cur.getX() - this.prev.getX();
      cYpY = this.cur.getY() - this.prev.getY();

      return cXpX * cXpX + cYpY * cYpY;
    }
    this.draw = function(ctx, scaleFactor)
    {
      ctx.lineWidth = 5;
      ctx.fillStyle = '#000000';
      ctx.strokeStyle = '#000000';
      ctx.beginPath();
      ctx.arc(this.cur.getX() * scaleFactor,
              this.cur.getY() * scaleFactor,
              6.0, 0.0, Math.PI * 2.0, true);
      ctx.fill();
    }
  }

  function Joint(pointMassA, pointMassB, shortConst, longConst)
  {
    this.pointMassA = pointMassA;
    this.pointMassB = pointMassB;
    this.delta = new Vector(0.0, 0.0);
    this.pointMassAPos = pointMassA.getPos();
    this.pointMassBPos = pointMassB.getPos();

    this.delta.set(this.pointMassBPos);
    this.delta.sub(this.pointMassAPos);

    this.shortConst = this.delta.length() * shortConst;
    this.longConst = this.delta.length() * longConst;
    this.scSquared = this.shortConst * this.shortConst;
    this.lcSquared = this.longConst * this.longConst;

    this.middle = new Vector();
    this.t = new Vector();
    this.k = new Vector();

    this.setDist = function(shortConst, longConst)
    {
      this.shortConst = shortConst;
      this.longConst = longConst;
      this.scSquared = this.shortConst * this.shortConst;
      this.lcSquared = this.longConst * this.longConst;
    }
    this.scale = function(scaleFactor)
    {
      this.shortConst = this.shortConst * scaleFactor;
      this.longConst = this.longConst * scaleFactor;
      this.scSquared = this.shortConst * this.shortConst;
      this.lcSquared = this.longConst * this.longConst;
    }
    this.sc = function()
    {
      this.delta.set(this.pointMassBPos);
      this.delta.sub(this.pointMassAPos);

      var dp = this.delta.dotProd(this.delta);
      if(this.shortConst != 0.0 && dp < this.scSquared)
      {
        var scaleFactor;
        scaleFactor = this.scSquared / (dp + this.scSquared) - 0.5;
        this.delta.scale(scaleFactor);
        this.pointMassAPos.sub(this.delta);
        this.pointMassBPos.add(this.delta);
      }
      else if(this.longConst != 0.0 && dp > this.lcSquared)
      {
        var scaleFactor;
        scaleFactor = this.lcSquared / (dp + this.lcSquared) - 0.5;
        this.delta.scale(scaleFactor);
        this.pointMassAPos.sub(this.delta);
        this.pointMassBPos.add(this.delta);
      }
    }
    this.draw = function(ctx, scaleFactor)
    {
      this.middle.set(this.pointMassA.getPos());
      this.t.set(this.pointMassB.getPos());
      this.t.sub(this.middle);
      this.t.scale(0.5);
      this.middle.add(this.t);

      this.t.set(this.middle);
      this.t.sub(this.pointMassA.getPos());

      ctx.lineWidth = 2;

      // long const bar
      this.t.normalize();
      this.t.scale(this.longConst * 0.5);
      var t1 = this.middle.getX() + this.t.getX();
      var t2 = this.middle.getY() + this.t.getY();
      var t3 = this.middle.getX() - this.t.getX();
      var t4 = this.middle.getY() - this.t.getY();

      ctx.strokeStyle = '#aaaaaa';
      ctx.beginPath();
      ctx.moveTo(t1 * scaleFactor, t2 * scaleFactor);
      ctx.lineTo(t3 * scaleFactor, t4 * scaleFactor);
      ctx.stroke();

      this.k.set(this.t);
      this.k.normalize();
      this.k.scale(0.03);
      var x = -this.k.getY();
      var y = this.k.getX();
      ctx.strokeStyle = '#0000ff';
      ctx.beginPath();
      ctx.moveTo((t1 + x) * scaleFactor, (t2 + y) * scaleFactor);
      ctx.lineTo((t1 - x) * scaleFactor, (t2 - y) * scaleFactor);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo((t3 + x) * scaleFactor, (t4 + y) * scaleFactor);
      ctx.lineTo((t3 - x) * scaleFactor, (t4 - y) * scaleFactor);
      ctx.stroke();

      // short const bar
      this.t.normalize();
      this.t.scale(this.shortConst * 0.5);

      t1 = this.middle.getX() + this.t.getX();
      t2 = this.middle.getY() + this.t.getY();
      t3 = this.middle.getX() - this.t.getX();
      t4 = this.middle.getY() - this.t.getY();

      ctx.strokeStyle = '#ff0000';
      ctx.beginPath();
      ctx.moveTo((t1 + x) * scaleFactor, (t2 + y) * scaleFactor);
      ctx.lineTo((t1 - x) * scaleFactor, (t2 - y) * scaleFactor);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo((t3 + x) * scaleFactor, (t4 + y) * scaleFactor);
      ctx.lineTo((t3 - x) * scaleFactor, (t4 - y) * scaleFactor);
      ctx.stroke();
    }
  }

  var env;
  var width = 600.0;
  var height = 400.0;
  var scaleFactor = 200.0;
  var blobColl;
  var gravity;
  var stopped;
  var savedMouseCoords = null;
  var selectOffset = null;

  function update()
  {
    var dt = 0.05;

    if(savedMouseCoords != null && selectOffset != null)
    {
      blobColl.selectedBlobMoveTo(savedMouseCoords.x - selectOffset.x,
        savedMouseCoords.y - selectOffset.y);
    }

    blobColl.move(dt);
    blobColl.sc(env);
    blobColl.setForce(gravity);
  }

  function draw()
  {
    var canvas = document.getElementById('blob');
    if(canvas.getContext == null)
    {
      return;
    }

    var ctx = canvas.getContext('2d');

    ctx.clearRect(0, 0, width, height);

    env.draw(ctx, scaleFactor);
    blobColl.draw(ctx, scaleFactor);
  }

  function timeout()
  {
    draw();
    update();

    if(stopped == false)
    {
      setTimeout('timeout()', 30);
    }
  }

  function init()
  {
    var canvas = document.getElementById('blob');
    if(canvas.getContext == null)
    {
      alert("You need Firefox version 1.5 or higher for this to work, sorry.");
      return;
    }

    document.onkeydown = function(event)
    {
      var keyCode;

      if(event == null)
      {
        keyCode = window.event.keyCode;
      }
      else
      {
        keyCode = event.keyCode;
      }

      switch(keyCode)
      {
        // left
        case 37:
          blobColl.addForce(new Vector(-50.0, 0.0));
          break;

        // up
        case 38:
          blobColl.addForce(new Vector(0.0, -50.0));
          break;

        // right
        case 39:
          blobColl.addForce(new Vector(50.0, 0.0));
          break;

        // down
        case 40:
          blobColl.addForce(new Vector(0.0, 50.0));
          break;

        // join 'j'
        case 74:
          blobColl.join();
          break;

        // split 'h'
        case 72:
          blobColl.split();
          break;

        // toggle gravity 'g'
        case 71:
          toggleGravity();
          break;

        default:
          break;
      }
    }


    function getMouseCoords(event)
    {
      if(event == null)
      {
        event = window.event;
      }
      if(event == null)
      {
        return null;
      }
      if(event.pageX || event.pageY){
        return {x:event.pageX / scaleFactor, y:event.pageY / scaleFactor};
      }
      return null;
    }
    document.onmousedown = function(event)
    {
      var mouseCoords;

      if(stopped == true)
      {
        return;
      }
      mouseCoords = getMouseCoords(event);
      if(mouseCoords == null)
      {
        return;
      }
      selectOffset = blobColl.selectBlob(mouseCoords.x, mouseCoords.y);
    }
    document.onmouseup = function(event)
    {
      blobColl.unselectBlob();
      savedMouseCoords = null;
      selectOffset = null;
    }
    document.onmousemove = function(event)
    {
      var mouseCoords;

      if(stopped == true)
      {
        return;
      }
      if(selectOffset == null)
      {
        return;
      }
      mouseCoords = getMouseCoords(event);
      if(mouseCoords == null)
      {
        return;
      }
      blobColl.selectedBlobMoveTo(mouseCoords.x - selectOffset.x, mouseCoords.y - selectOffset.y);

      savedMouseCoords = mouseCoords;
    }

    env = new Environment(0.2, 0.2, 2.6, 1.6);
    blobColl = new BlobCollective(1.0, 1.0, 1, 200);
    gravity = new Vector(0.0, 10.0);
    stopped = false;

    timeout();
  }

  function stop()
  {
    stopped = true;
  }
  function start()
  {
    stopped = false;
    timeout();
  }
  function toggleGravity()
  {
    if(gravity.getY() > 0.0)
    {
      gravity.setY(0.0);
    }
    else
    {
      gravity.setY(10.0);
    }
  }

