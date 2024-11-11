// 2015-07-29 xoihazard

!function(){
  'use strict';

  var Model = function() {
    this.init.apply(this, arguments);
  };

  Model.prototype = {

    init: function() {
      var this_ = this;

      this.param = {

        unsaved: {
          canvasSize: 1000,
          noiseEnable: false,
          noiseScale: 1,
          noiseStrength: 30,
          noiseDisplace: 0.1,
          noiseOffset: 0,
          noiseRepeat: 10
        },

        srcBlur: 5,
        srcWarp: 1,
        srcGamma: 1,
        srcClampMin: 0.05,
        srcClampMax: 0.95,

        gridSpace: 20,
        gridAngle: 0,
        gridRadius: 1,

        dotRescale: 1,
        dotClipMax: 1,
        dotClipMin: 0,
        dotNegative: false,
        dotAffect: 1,

        shapeStyle: 'Circle',
        shapeOutline: false,

        channel: 'Black',
        background: '#ffffff',
        foreground: '#000000',
        alpha: 1,
        transparent: false,
        decompose: false,
        reverse: false,
        sort: 'None',

        channel1: 'Red',
        channel2: 'Green',
        channel3: 'Blue',
        color1: '#ff0000',
        color2: '#00ff00',
        color3: '#0000ff'
      };

      this.chNumbers = {
        None: -1,
        Luminance: 4,
        Red: 0,
        Green: 1,
        Blue: 2,
        //Alpha: 3,
        Cyan: 5,
        Magenta: 6,
        Yellow: 7,
        Black: 8
      };

      this.canvas = document.createElement('canvas');
      this.ctx = this.canvas.getContext('2d');

      // for StackBlur.js
      this.canvas.id = 'source';
      this.canvas.setAttribute('style', 'display: none;');
      document.body.appendChild(this.canvas);

      this.canvas_o = document.createElement('canvas');
      this.ctx_o = this.canvas_o.getContext('2d');

      this.shape = new Shape();
      this.shape.setContext(this.ctx_o);

      this.img = new Image();
      this.img.onload = function() {
        this_.source = this_.img;
        this_.sourceLoaded = true;
        this_.onChangeImage();
        if (this_.hasOwnProperty('onloadCallback')) {
          if (typeof(this_.onloadCallback === 'function')) {
            this_.onloadCallback(this_);
          }
        }
      };

      this.drawpoints = [];
      this.source = this.img;
      this.sourceLoaded = false;
    },

    onChangeGrid: function() {
      this.updateGrid();
      this.updateRadius(true);
      this.drawCanvas();
    },

    onChangeGridAngle: function() {
      this.updateGridAngle();
      this.updateRadius(true);
      this.drawCanvas();
    },

    onChangeSample: function() {
      this.updateBlur();
      this.updateRadius();
      this.drawCanvas();
    },

    onChangeRadius: function(turbulence) {
      this.updateRadius(turbulence);
      this.drawCanvas();
    },

    onChangeNoise: function() {
      if (!this.param.unsaved.noiseEnable) return;

      this.onChangeRadius(true);
    },

    onChangeStyle: function() {
      this.updateShape();
      this.drawCanvas();
    },

    onChangeImage: function() {
      this.updateShape();
      this.updateSource();
      this.updateGrid();
      this.updateRadius(true);
      this.drawCanvas();
    },

    updateSource: function() {
      if (!this.sourceLoaded) return;

      var sw = this.param.unsaved.canvasSize / this.source.width,
          sh = this.param.unsaved.canvasSize / this.source.height,
          ow,
          oh,
          scale;

      scale = Math.min(sw, sh);
      ow = Math.round(this.source.width * scale);
      oh = Math.round(this.source.height * scale);

      this.canvas.width = this.canvas_o.width = ow;
      this.canvas.height = this.canvas_o.height = oh;

      this.updateBlur();
    },

    updateBlur: function() {
      if (!this.sourceLoaded) return;

      var w = this.canvas.width,
          h = this.canvas.height,
          radius = this.param.srcBlur;

      this.ctx.clearRect(0, 0, w, h);
      this.ctx.drawImage(this.source, 0, 0, w, h);
      stackBlurCanvasRGB(this.canvas.id, 0, 0, w, h, radius);

      this.imageData = this.ctx.getImageData(0, 0, w, h);
    },

    updateGrid: function() {
      this.hexGrid = new HexGrid(
        Math.max(this.canvas.width, this.canvas.height) /
        this.param.gridSpace *
        this.param.gridRadius
      );
      this.updateGridAngle();
    },

    updateGridAngle: function() {
      var angle_radian = this.param.gridAngle / 360 * Math.PI * 2;
      this.hexGrid.projection(
        this.canvas.width / 2,
        this.canvas.height / 2,
        this.param.gridSpace / 2,
        angle_radian
      );
    },

    updateRadius: function(turbulence) {
      if (!this.sourceLoaded) return;

      var this_ = this,
          w = this.canvas.width,
          h = this.canvas.height,
          ch, level;

      this.drawpoints.length = 0;

      for (var i = 0; i < this.hexGrid.points.length; i++) {
        var hex = this.hexGrid.points[i];

        // canvas contains p
        if (hex.x >= 0 && hex.x <= w && hex.y >= 0 && hex.y <= h) {

          if (this.param.decompose) {
            switch (hex.type) {
              case 0:
                ch = this.chNumbers[this.param.channel1];
                break;
              case 1:
                ch = this.chNumbers[this.param.channel2];
                break;
              case 2:
                ch = this.chNumbers[this.param.channel3];
                break;
            }
          } else {
            ch = this.chNumbers[this.param.channel];
          }

          // if not selected a channel
          if (ch < 0) continue;

          level = (this.param.dotNegative) ? 1 - this.getLevel(hex.x, hex.y, ch) : this.getLevel(hex.x, hex.y, ch);

          if (turbulence) {
            if (this.param.unsaved.noiseEnable) {
              this.turbulence(hex, 1);
            } else {
              hex.renderX = hex.x;
              hex.renderY = hex.y;
            }
          }

          // clip
          if (level >= this.param.dotClipMin && level <= this.param.dotClipMax) {
            hex.renderRadius = (1 - (1 - level) * this.param.dotAffect) * this.param.gridSpace / 2 * this.param.dotRescale;

            this.drawpoints.push(hex);
          }
        }
      }

      if (this.param.sort !== 'None') {
        this.drawpoints.sort(this.sortMethods[this.param.sort]);
      }

    },

    smoothingTurbulence: function() {
      if (!this.sourceLoaded || !this.param.unsaved.noiseEnable) return;

      for (var i = 0; i < this.hexGrid.points.length; i++) {
        var hex = this.hexGrid.points[i];
        this.turbulence(hex, this.param.unsaved.noiseRepeat);
      }
    },

    updateShape: function() {
      this.shape.setDrawMethod(this.param.shapeStyle);
    },

    turbulence: function(hex, repeat) {
      if (this.param.unsaved.noiseDisplace === 0 || this.param.unsaved.noiseScale === 0) {
        hex.renderX = hex.x;
        hex.renderY = hex.y;
        return;
      }

      var x = hex.x,
          y = hex.y,
          angle;

      for (var i = 0; i < repeat; i++) {
        angle = noise(
          (x - this.canvas.width * 0.5) / (this.param.unsaved.canvasSize * this.param.unsaved.noiseScale),
          (y - this.canvas.height * 0.5) / (this.param.unsaved.canvasSize * this.param.unsaved.noiseScale),
          this.param.unsaved.noiseOffset
        ) * this.param.unsaved.noiseStrength;

        x += Math.cos(angle) *
          this.param.unsaved.noiseDisplace *
          this.param.unsaved.canvasSize *
          this.param.unsaved.noiseScale /
          repeat;

        y += - Math.sin(angle) *
          this.param.unsaved.noiseDisplace *
          this.param.unsaved.canvasSize *
          this.param.unsaved.noiseScale /
          repeat;
      }

      hex.renderX = x;
      hex.renderY = y;
    },

    drawSVG: function() {
      if (!this.sourceLoaded) return;

      var color,
          index;

      this.shape.setAlpha(this.param.alpha);

      if (!this.param.transparent) {
        this.shape.setFill(this.param.background);
        this.shape.fillSVG(0, 0, this.canvas.width, this.canvas.height);
      }

      for (var i = 0; i < this.drawpoints.length; i++) {
        index = (this.param.reverse) ? this.drawpoints.length - i - 1 : i;

        if (this.param.decompose) {
          switch (this.drawpoints[index].type) {
            case 0:
              color = this.param.color1;
              break;
            case 1:
              color = this.param.color2;
              break;
            case 2:
              color = this.param.color3;
              break;
          }
        } else {
          color = this.param.foreground;
        }

        if (this.param.shapeOutline) {
          this.shape.setStroke(color, 1);
        } else {
          this.shape.setFill(color);
        }

        this.shape.drawSVG(this.drawpoints[index]);
      }
    },

    drawCanvas: function() {
      if (!this.sourceLoaded) return;

      var this_ = this,
          startIndex = 0,
          lastUpdate = Date.now(),
          color;

      if (this.requestID) {
        cancelAnimationFrame(this.requestID);
      }

      this.shape.setAlpha(this.param.alpha);

      if (this.param.transparent) {
        this.ctx_o.clearRect(0, 0, this.canvas_o.width, this.canvas_o.height);
      } else {
        this.ctx_o.fillStyle = this.param.background;
        this.ctx_o.globalAlpha = 1;
        this.ctx_o.fillRect(0, 0, this.canvas_o.width, this.canvas_o.height);
      }

      function loop() {
        var start = Date.now(),
            timeover = false,
            now,
            index;

        for (var i = startIndex; i < this_.drawpoints.length; i++) {
          now = Date.now();
          index = (this_.param.reverse) ? this_.drawpoints.length - i - 1 : i;

          if (startIndex === 0 && now - lastUpdate > 1000 / 30) {
            startIndex = i;
            timeover = true;
            lastUpdate = Date.now();
            break;
          } else if (startIndex > 0 && (i - startIndex > 1000 || now - lastUpdate < 200)) {
            startIndex = i;
            timeover = true;
            break;
          }

          if (this_.param.decompose) {
            switch (this_.drawpoints[index].type) {
              case 0:
                color = this_.param.color1;
                break;
              case 1:
                color = this_.param.color2;
                break;
              case 2:
                color = this_.param.color3;
                break;
            }
          } else {
            color = this_.param.foreground;
          }

          if (this_.param.shapeOutline) {
            this_.shape.setStroke(color, 1);
          } else {
            this_.shape.setFill(color);
          }

          this_.shape.drawCanvas(this_.drawpoints[index]);
        }

        if (timeover) {
          this_.requestID = requestAnimationFrame(loop);
        }
      }

      loop();
    },

    getChNames: function() {
      var a = [];
      for (var key in this.chNumbers) {
        a.push(key);
      }
      return a;
    },

    getSortMethodNames: function() {
      var a = ['None'];
      for (var key in this.sortMethods) {
        a.push(key);
      }
      return a;
    },

    getLevel: function(x, y, ch) {
      if (ch < 0) return 0;

      x = x | 0;
      y = y | 0;

      var w = this.imageData.width,
          l = 0,
          R = this.imageData.data[(x + y * w) * 4 + 0],
          G = this.imageData.data[(x + y * w) * 4 + 1],
          B = this.imageData.data[(x + y * w) * 4 + 2],
          A = this.imageData.data[(x + y * w) * 4 + 3];

      switch (ch) {
        case 0: // Red
          l = R;
          break;
        case 1: // Green
          l = G;
          break;
        case 2: // Blue
          l = B;
          break;
        case 3: // Alpha
          l = A;
          break;
        case 4: // Luminance
          l = (0.299 * R + 0.587 * G + 0.114 * B);
          break;
        case 5: // Cyan
          l = (255 - R);
          break;
        case 6: // Magenta
          l = (255 - G);
          break;
        case 7: // Yellow
          l = (255 - B);
          break;
        case 8: // Black
          l = 255 - (0.299 * R + 0.587 * G + 0.114 * B);
          break;
      }

      l = l /255;

      // clamp and normalize
      if (l <= this.param.srcClampMin) {
        l = 0;
      } else if (l >= this.param.srcClampMax) {
        l = 1;
      } else {
        l = (l - this.param.srcClampMin) / (this.param.srcClampMax - this.param.srcClampMin);
      }

      // gamma
      if (this.param.srcGamma !== 1) {
        l = Math.pow(l, this.param.srcGamma);
      }

      // warp
      if (this.param.srcWarp !== 1 && l < 1) {
        l = l * this.param.srcWarp % 1;
      }

      return l;
    },

    sortMethods: {

      'Radius' : function(a, b) {
        if (a.renderRadius > b.renderRadius) return -1;
        if (a.renderRadius < b.renderRadius) return 1;
        return 0;
      },

      'X' : function(a, b) {
        if (a.x < b.x) return -1;
        if (a.x > b.x) return 1;
        return 0;
      },

      'Y' : function(a, b) {
        if (a.y < b.y) return -1;
        if (a.y > b.y) return 1;
        return 0;
      },

      'Type' : function(a, b) {
        if (a.type < b.type) return -1;
        if (a.type > b.type) return 1;
        return 0;
      }

    },

    download: function(blob, extention) {
      if (this.blobURL) URL.revokeObjectURL(this.blobURL);
      this.blobURL = URL.createObjectURL(blob);
      var unixtime = Math.floor(new Date().getTime() / 1000);

      this.downloadLink = document.createElement('a');
      this.downloadLink.href = this.blobURL;
      this.downloadLink.download = 'halftoned_' + unixtime + '.' + extention;
      this.downloadLink.target = '_blank';

      var e = document.createEvent('MouseEvents');
      e.initMouseEvent(
        'click',
        true,
        true,
        window,
        0,
        0,
        0,
        0,
        0,
        false,
        false,
        false,
        false,
        0,
        this.downloadLink
      );

      this.downloadLink.dispatchEvent(e);
    },

    downloadAsPNG: function() {
      var dataURL = this.canvas_o.toDataURL('image/png');
      this.download(dataURLtoBlob(dataURL), 'png');
    },

    downloadAsSVG: function() {
      var xs = new XMLSerializer(),
          svg = document.createElementNS(SVG_NS, 'svg'),
          svgStr,
          blob,
          w = this.canvas.width,
          h = this.canvas.height;

      svg.setAttribute('version', '1.1');
      svg.setAttribute('xmlns', SVG_NS);
      svg.setAttribute('width', w + 'px');
      svg.setAttribute('height', h + 'px');
      svg.setAttribute('x', '0');
      svg.setAttribute('y', '0');
      svg.setAttribute('viewBox', [0, 0, w, h].join(' '));

      this.shape.setSVG(svg);
      this.drawSVG();

      svgStr = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n" + xs.serializeToString(svg);
      blob = new Blob([svgStr], {type: 'image/svg+xml'});

      this.download(blob, 'svg');
    }

  };

  var Shape = function() {
    this.init.apply(this, arguments);
  };

  Shape.prototype = {

    init: function() {
      this.setDrawMethod('Circle');
      this.alpha = 1;
    },

    setSVG: function(svg) {
      this.svg = svg;
      this.svgGroup = document.createElementNS(SVG_NS, 'g'),
      this.svg.appendChild(this.svgGroup);
    },

    setContext: function(ctx) {
      this.ctx = ctx;

      this.ctx.globalAlpha = 0.5;
    },

    setFill: function(fillStyle) {
      this.fillStyle = fillStyle;
      this.fill = true;
    },

    setStroke: function(strokeStyle, lineWidth) {
      this.strokeStyle = strokeStyle;
      this.lineWidth = lineWidth;
      this.fill = false;
    },

    drawMethods: {

      'Circle': {
        canvas: function(hex) {
          this.ctx.beginPath();
          this.ctx.arc(hex.renderX, hex.renderY, hex.renderRadius, 0, Math.PI * 2, false);
          this.ctx.closePath();
        },
        svg: function(hex) {
          var el = document.createElementNS(SVG_NS, 'circle');
          el.setAttribute('cx', hex.renderX);
          el.setAttribute('cy', hex.renderY);
          el.setAttribute('r', hex.renderRadius);
          return el;
        }
      },

      'Incircle': {
        canvas: function(hex) {
          this.ctx.beginPath();
          this.ctx.arc(hex.renderX, hex.renderY, hex.renderRadius * Math.sqrt(3) / 2, 0, Math.PI * 2, false);
          this.ctx.closePath();
        },
        svg: function(hex) {
          var el = document.createElementNS(SVG_NS, 'circle');
          el.setAttribute('cx', hex.renderX);
          el.setAttribute('cy', hex.renderY);
          el.setAttribute('r', hex.renderRadius * Math.sqrt(3) / 2);
          return el;
        }
      },

      'Triangle': {
        canvas: function(hex) {
          if (hex.type == 0) {
            throw true;
            return;
          }
          var offset = (hex.type == 1) ? 0.5 : 0;
          this.ctx.beginPath();
          for (var i = 0; i < 3; i++) {
            var a = hex.angle + Math.PI * 2 / 3 * (i + offset),
                x = hex.renderX + hex.renderRadius * Math.cos(a) / Math.sqrt(3) * 3,
                y = hex.renderY + hex.renderRadius * Math.sin(a) / Math.sqrt(3) * 3;

            if (i === 0) {
              this.ctx.moveTo(x, y);
            } else {
              this.ctx.lineTo(x, y);
            }
          }
          this.ctx.closePath();
        },
        svg: function(hex) {
          if (hex.type == 0) {
            throw true;
            return;
          }
          var offset = (hex.type == 1) ? 0.5 : 0;
          var points = [];
          for (var i = 0; i < 3; i++) {
            var a = hex.angle + Math.PI * 2 / 3 * (i + offset),
                x = hex.renderX + hex.renderRadius * Math.cos(a) / Math.sqrt(3) * 3,
                y = hex.renderY + hex.renderRadius * Math.sin(a) / Math.sqrt(3) * 3;

            points.push(x + ' ' + y);
          }
          var el = document.createElementNS(SVG_NS, 'polygon');
          el.setAttribute('points', points.join(', '));
          return el;
        }
      },

      'Hexagon': {
        canvas: function(hex) {
          this.ctx.beginPath();
          for (var i = 0; i < 6; i++) {
            var a = hex.angle + Math.PI * 2 / 6 * (i + 0.5),
                x = hex.renderX + hex.renderRadius * Math.cos(a),
                y = hex.renderY + hex.renderRadius * Math.sin(a);

            if (i === 0) {
              this.ctx.moveTo(x, y);
            } else {
              this.ctx.lineTo(x, y);
            }
          }
          this.ctx.closePath();
        },
        svg: function(hex) {
          var points = [];
          for (var i = 0; i < 6; i++) {
            var a = hex.angle + Math.PI * 2 / 6 * (i + 0.5),
                x = hex.renderX + hex.renderRadius * Math.cos(a),
                y = hex.renderY + hex.renderRadius * Math.sin(a);

            points.push(x + ' ' + y);
          }
          var el = document.createElementNS(SVG_NS, 'polygon');
          el.setAttribute('points', points.join(', '));
          return el;
        }
      },

      'Hexagram': {
        canvas: function(hex) {
          this.ctx.beginPath();
          var r;
          for (var i = 0; i < 12; i++) {
            var a = hex.angle + Math.PI * 2 / 12 * (i + 1),
                r = (i % 2 > 0) ? hex.renderRadius / Math.sqrt(3) : hex.renderRadius,
                x = hex.renderX + r * Math.cos(a),
                y = hex.renderY + r * Math.sin(a);

            if (i === 0) {
              this.ctx.moveTo(x, y);
            } else {
              this.ctx.lineTo(x, y);
            }
          }
          this.ctx.closePath();
        },
        svg: function(hex) {
          var r, points = [];
          for (var i = 0; i < 12; i++) {
            var a = hex.angle + Math.PI * 2 / 12 * (i + 1),
                r = (i % 2 > 0) ? hex.renderRadius / Math.sqrt(3) : hex.renderRadius,
                x = hex.renderX + r * Math.cos(a),
                y = hex.renderY + r * Math.sin(a);

            points.push(x + ' ' + y);
          }
          var el = document.createElementNS(SVG_NS, 'polygon');
          el.setAttribute('points', points.join(', '));
          return el;
        }
      }

    },

    getDrawMethodNames: function() {
      var a = [];
      for (var key in this.drawMethods) {
        a.push(key);
      }
      return a;
    },

    setAlpha: function(value) {
      this.alpha = value;
    },

    setDrawMethod: function(name) {
      if (this.drawMethods.hasOwnProperty(name)) {
        this.drawMethod = this.drawMethods[name];
      }
    },

    drawCanvas: function(hex) {
      if (hex.renderRadius <= 0) return;

      try {
        this.drawMethod.canvas.apply(this, arguments);
      } catch (e) {
        return;
      }

      this.ctx.globalAlpha = this.alpha;

      if (this.fill) {
        this.ctx.fillStyle = this.fillStyle;
        this.ctx.fill();
      } else {
        this.ctx.strokeStyle = this.strokeStyle;
        this.ctx.lineWidth = this.lineWidth;
        this.ctx.stroke();
      }
    },

    drawSVG: function(hex) {
      if (hex.renderRadius <= 0) return;

      var el;

      try {
        el = this.drawMethod.svg.apply(this, arguments);
      } catch (e) {
        return;
      }

      if (el) {
        if (this.fill) {
          el.setAttribute('fill', this.fillStyle);
          el.setAttribute('fill-opacity', this.alpha);
        } else {
          el.setAttribute('fill', 'none');
          el.setAttribute('stroke', this.strokeStyle);
          el.setAttribute('stroke-opacity', this.alpha);
        }
        this.svgGroup.appendChild(el);
      }
    },

    fillSVG: function(x, y, w, h) {
      var el = document.createElementNS(SVG_NS, 'rect');
      el.setAttribute('fill', this.fillStyle);
      el.setAttribute('x', x);
      el.setAttribute('y', y);
      el.setAttribute('width', w);
      el.setAttribute('height', h);
      this.svgGroup.appendChild(el);
      this.svgGroup = document.createElementNS(SVG_NS, 'g'),
      this.svg.appendChild(this.svgGroup);
    }
  };

  var HexGrid = function() {
    this.init.apply(this, arguments);
  };

  HexGrid.prototype = {

    init: function(radius) {
      radius = radius || 0;
      this.neighbors = [
        [1, 0],
        [1, -1],
        [0, -1],
        [-1, 0],
        [-1, 1],
        [0, 1]
      ];
      this.make(radius);
    },

    getNeighborHex: function(hex, direction) {
      direction = ~~direction % 6;
      return new Hex(
        hex.q + this.neighbors[direction][0],
        hex.r + this.neighbors[direction][1]
      );
    },

    make: function(radius) {
      var this_ = this;
      this.points = [];
      function getHexRing(radius) {
        var res = [],
            hex = new Hex(-radius, radius);
        if (radius === 0) {
          res.push(hex);
        } else {
          for (var i = 0; i < 6; i++) {
            for (var j = 0; j < radius; j++) {
              res.push(hex);
              hex = this_.getNeighborHex(hex, i);
            }
          }
        }
        return res;
      }
      for (var i = 0; i <= radius; i++) {
        Array.prototype.push.apply(this.points, getHexRing(i));
      }
    },

    projection: function(cx, cy, unitsize, angle) {
      for (var i = 0; i < this.points.length; i++) {
        this.points[i].projection(cx, cy, unitsize, angle);
      }
    }
  };

  var Hex = function() {
    this.init.apply(this, arguments);
  };

  Hex.prototype = {
    init: function(q, r) {
      this.q = q || 0;
      this.r = r || 0;
      this.s = - this.q - this.r;
      if ((this.q - this.r) % 3 == 0) {
        this.type = 0;
      } else if ((this.q - this.r - 1) % 3 == 0) {
        this.type = 1;
      } else {
        this.type = 2;
      }
      //this.mag = Math.sqrt(this.q * this.q + this.r * this.r + this.s * this.s);
    },

    projection: function(cx, cy, unitsize, angle) {
      cx = cx || 0;
      cy = cy || 0;
      unitsize = unitsize || 10;
      angle = angle || 0;

      var x = unitsize * Math.sqrt(3) * (this.q + this.r / 2),
          y = unitsize * 3 / 2 * this.r;

      // rotate & translate x, y
      this.angle = angle;
      this.x = x * Math.cos(angle) - y * Math.sin(angle) + cx;
      this.y = y * Math.cos(angle) + x * Math.sin(angle) + cy;

      this.renderX = this.x;
      this.renderY = this.y;
    }

  };

  var UI = function() {
    this.init.apply(this, arguments);
  };

  UI.prototype = {

    init: function(element, presetURL) {
      var this_ = this;

      this.model = new Model();
      this.cropper = new Cropper(this.model);
      element.appendChild(this.model.canvas_o);

      getJSON(presetURL, function(data) {
        this_.presets = data;
        this_.createGUI();
      }, function(status) {
        this_.createGUI();
      });
    },

    setSource: function(url) {
      this.model.img.src = url;
    },

    createGUI: function() {

      var model =this.model,
          cropper = this.cropper,
          this_ = this;

      // file reader
      var fileinput = document.createElement('input');
      fileinput.type = 'file';
      fileinput.accept = 'image/*';

      fileinput.addEventListener('change', function(event) {
        var reader = new FileReader();
        reader.onload = function(e) {
          var dataURL = e.target.result;
          model.img.src = dataURL;
        };
        if (event.target.files.length > 0) {
          reader.readAsDataURL(event.target.files[0]);
        }
      }, false);

      // dat.GUI
      var gui = new dat.GUI({
        load: this_.presets,
        width: 300,
        scrollable: false
      });

      gui.remember(model.param);

      gui.onChangePreset(function() {
        model.onChangeImage();
      });

      // 

      gui.add({f: function() {
        fileinput.click();
      }}, 'f')
        .name('Open image...');

      gui.add({f: function() {
        cropper.show();
      }}, 'f')
        .name('Crop image...');

      gui.add(model.param.unsaved, 'canvasSize', 100, 2000)
        .name('Canvas size')
        .step(100)
        .onFinishChange(function() {
          model.onChangeImage();
        });

      // Color correction

      var f2 = gui.addFolder('Color correction');
      f2.open();

      f2.add(model.param, 'srcClampMin', 0, 1)
        .name('Clamp (Min)')
        .step(0.01)
        .onChange(function() {
          model.onChangeRadius();
        });

      f2.add(model.param, 'srcClampMax', 0, 1)
        .name('Clamp (max)')
        .step(0.01)
        .onChange(function() {
          model.onChangeRadius();
        });

      f2.add(model.param, 'srcGamma', 0.01, 10)
        .name('Gamma')
        .step(0.05)
        .mid(1)
        .onChange(function() {
          model.onChangeRadius();
        });

      // Source effects

      var f1 = gui.addFolder('Source effects');
      f1.open();

      f1.add(model.param, 'srcBlur', 0, 100)
        .name('Blur')
        .step(1)
        .onFinishChange(function() {
          model.onChangeSample();
        });

      f1.add(model.param, 'srcWarp', 1, 10)
        .name('Warp')
        .step(0.01)
        .onChange(function() {
          model.onChangeRadius();
        });

      var f3 = gui.addFolder('Grid');
      f3.open();

      f3.add(model.param, 'gridSpace', 3, 100)
        .name('Space')
        .step(1)
        .onChange(function() {
          model.onChangeGrid();
        });

      f3.add(model.param, 'gridAngle', -60, 60)
        .name('Angle')
        .step(1)
        .onChange(function() {
          model.onChangeGridAngle();
        });

      f3.add(model.param, 'gridRadius', 0, 1)
        .name('Radius')
        .step(0.01)
        .onChange(function() {
          model.onChangeGrid();
        });

      // Dot

      var f4 = gui.addFolder('Dot');
      f4.open();

      f4.add(model.param, 'channel', model.getChNames())
        .name('Channel')
        .onChange(function() {
          model.onChangeRadius();
        });

      f4.add(model.param, 'dotNegative')
        .name('Negative')
        .onChange(function() {
          model.onChangeRadius();
        });

      f4.add(model.param, 'dotClipMin', 0, 1)
        .name('Clip (Min)')
        .step(0.01)
        .onChange(function() {
          model.onChangeRadius();
        });

      f4.add(model.param, 'dotClipMax', 0, 1)
        .name('Clip (Max)')
        .step(0.01)
        .onChange(function() {
          model.onChangeRadius();
        });

      f4.add(model.param, 'dotAffect', 0, 1)
        .name('Affect size')
        .step(0.01)
        .onChange(function() {
          model.onChangeRadius();
        });

      f4.add(model.param, 'dotRescale', 0.1, 20)
        .name('Rescale')
        .step(0.1)
        .onChange(function() {
          model.onChangeRadius();
        });

      // Shape

      var f5 = gui.addFolder('Shape');
      f5.open();

      f5.add(model.param, 'shapeStyle', model.shape.getDrawMethodNames())
        .name('Style')
        .onChange(function() {
          model.onChangeStyle();
        });

      f5.add(model.param, 'shapeOutline')
        .name('Outline')
        .onChange(function() {
          model.onChangeStyle();
        });

      // Color

      var f6 = gui.addFolder('Color');
      f6.open();

      var guiFG, guiBG;

      f6.add(model.param, 'alpha', 0, 1)
        .name('Alpha')
        .step(0.01)
        .onChange(function() {
          model.onChangeStyle();
        });

      guiFG = f6.addColor(model.param, 'foreground')
        .name('Foreground')
        .onChange(function() {
          model.onChangeStyle();
        });

      guiBG = f6.addColor(model.param, 'background')
        .name('Background')
        .onChange(function() {
          model.onChangeStyle();
        });

      f6.add({f: function() {
        var bg = model.param.background,
            fg = model.param.foreground;
        guiFG.setValueOnly(bg);
        guiBG.setValueOnly(fg);
        model.onChangeStyle();
      }}, 'f')
        .name('Flip FG/BG');

      f6.add(model.param, 'transparent')
        .name('Transparent BG')
        .onChange(function() {
          model.onChangeStyle();
        });

      // Advanced

      var f7 = gui.addFolder('Decompose');

      f7.add(model.param, 'decompose')
        .name('Enable')
        .onChange(function() {
          model.onChangeRadius();
        });

      f7.add(model.param, 'sort', model.getSortMethodNames())
        .name('Sort')
        .onChange(function() {
          model.onChangeRadius();
        });

      f7.add(model.param, 'reverse')
        .name('Reverse draw')
        .onChange(function() {
          model.onChangeStyle();
        });

      f7.add(model.param, 'channel1', model.getChNames())
        .name('Channel 1')
        .onChange(function() {
          model.onChangeRadius();
        });

      f7.addColor(model.param, 'color1')
        .name('Color 1')
        .onChange(function() {
          model.onChangeStyle();
        });

      f7.add(model.param, 'channel2', model.getChNames())
        .name('Channel 2')
        .onChange(function() {
          model.onChangeRadius();
        });

      f7.addColor(model.param, 'color2')
        .name('Color 2')
        .onChange(function() {
          model.onChangeStyle();
        });

      f7.add(model.param, 'channel3', model.getChNames())
        .name('Channel 3')
        .onChange(function() {
          model.onChangeRadius();
        });

      f7.addColor(model.param, 'color3')
        .name('Color 3')
        .onChange(function() {
          model.onChangeStyle();
        });

      // Turbulence

      var f8 = gui.addFolder('Turbulence (those values are not saved to presets)');

      f8.add(model.param.unsaved, 'noiseEnable')
        .name('Enable')
        .onChange(function() {
          model.onChangeRadius(true);
        });

      f8.add(model.param.unsaved, 'noiseDisplace', -0.3, 0.3)
        .name('Displace')
        .step(0.01)
        .onChange(function() {
          model.onChangeNoise();
        });

      f8.add(model.param.unsaved, 'noiseOffset', 0, 5)
        .name('Offset')
        .step(0.01)
        .onChange(function() {
          model.onChangeNoise();
        });

      f8.add(model.param.unsaved, 'noiseScale', 0, 2)
        .name('Scale')
        .step(0.01)
        .onChange(function() {
          model.onChangeNoise();
        });

      f8.add(model.param.unsaved, 'noiseStrength', 0, 60)
        .name('Strangth')
        .step(1)
        .onChange(function() {
          model.onChangeNoise();
        });

      f8.add({f: function() {
          model.smoothingTurbulence();
          model.drawCanvas();
      }}, 'f')
        .name('Smooth');

      // downloads

      gui.add({f: function() {
        model.downloadAsPNG();
      }}, 'f')
        .name('Download PNG');

      gui.add({f: function() {
        model.downloadAsSVG();
      }}, 'f')
        .name('Download SVG');
    },

  };

  var Cropper = function() {
    this.init.apply(this, arguments);
  };

  Cropper.prototype = {

    init: function(model) {
      var this_ = this;

      this.model = model;
      this.el = document.createElement('div');
      this.el.className = 'modal';

      this.el.innerHTML =
        '<div class="modal-head">' +
        '<div class="modal-head-aspect">' +
        '<label>Aspect ratio: ' +
        '<select>' +
        '<option value="-1">original</option>' +
        '<option value="' + 1 + '" selected>1:1</option>' +
        '<option value="' + 4 / 3 + '">4:3</option>' +
        '<option value="' + Math.sqrt(2) + '">√2:1</option>' +
        '<option value="' + 3 / 2 + '">3:2</option>' +
        '<option value="' + (1 + Math.sqrt(5)) / 2 + '">&phi;:1</option>' +
        '<option value="' + 16 / 9 + '">16:9</option>' +
        '<option value="' + 2 + '">2:1</option>' +
        '<option value="' + 3 + '">3:1</option>' +
        '<option value="0">free</option>' +
        '</select>' +
        '</label> ' +
        '<label>' +
        '<input type="checkbox">' +
        'Invert</label>' +
        '</div>' +
        '<button class="button-clear">Clear</button> ' +
        '<button class="button-cancel">Cancel</button> ' +
        '<button class="button-apply">Crop</button>' +
        '</div>' +
        '<div class="modal-body"><div></div></div>';

      $('select, input', this.el).on('change', function() {
        this_.aspect();
      });

      $('.button-cancel', this.el).on('click', function() {
        this_.hide();
      });

      $('.button-clear', this.el).on('click', function() {
        this_.model.source = this_.model.img;
        this_.model.onChangeImage();
        this_.hide();
      });

      $('.button-apply', this.el).on('click', function() {
        this_.send();
        this_.hide();
      });

      document.body.appendChild(this.el);

      this.$el = $(this.el);
      this.$el_head = $('.modal-head', this.el);
      this.$el_body = $('.modal-body', this.el);
      this.$el_crop = $('.modal-body div', this.el);
      this.$el_crop.cropper({
        autoCropArea: 0.8,
        zoomable: false
      });

      this.aspect();
      this.hide();

      this.model.onloadCallback = function(model) {
        this_.resize();
        this_.aspect();
        this_.$el_crop.cropper('replace', model.img.src);
      };
    },

    aspect: function() {
      var value = $('select', this.el)[0].value,
          invert = ($('input', this.el)[0].checked) ? true : false;
      if (value != 0) {
        if (value < 0) {
          value = this.model.img.width / this.model.img.height;
        }
        value = (invert) ? 1 / value : value;
        this.$el_crop.cropper('setAspectRatio', value);
      } else {
        this.$el_crop.cropper('setAspectRatio', null);
      }
    },

    resize: function() {
      var this_ = this,
          ww = $(window).width(),
          wh = $(window).height(),
          iw = this.model.img.width,
          ih = this.model.img.height,
          sw, sh, ow, oh, size, scale;

      size = Math.min(ww, wh) / Math.sqrt(2);
      size = (size < 400) ? 400 : size;
      sw = size / iw;
      sh = size / ih;
      scale = Math.min(sw, sh);
      ow = Math.round(iw * scale);
      oh = Math.round(ih * scale);

      this.$el_body.width(ow);
      this.$el_body.height(oh);
    },

    send: function() {
      this.model.source = this.$el_crop.cropper('getCroppedCanvas');
      this.model.onChangeImage();
    },

    show: function() {
      this.resize();
      this.$el.fadeIn('fast');
      $(window).trigger('resize');
    },

    hide: function() {
      this.$el.fadeOut('fast');
    }

  };

  var SVG_NS = 'http://www.w3.org/2000/svg';

  var requestAnimationFrame =
    window.requestAnimationFrame ||
    window.mozRequestAnimationFrame ||
    window.webkitRequestAnimationFrame ||
    window.msRequestAnimationFrame;

  var cancelAnimationFrame =
    window.cancelAnimationFrame ||
    window.mozcancelAnimationFrame ||
    window.webkitcancelAnimationFrame ||
    window.mscancelAnimationFrame;

  function dataURLtoBlob(dataURL) {
    var byteString = atob(dataURL.split(',')[1]),
        mimeString = dataURL.split(',')[0].split(':')[1].split(';')[0],
        arrayBuffer = new ArrayBuffer(byteString.length),
        binaryArray = new Uint8Array(arrayBuffer),
        dataView = new DataView(arrayBuffer);

    for (var i = 0; i < byteString.length; i++) {
      binaryArray[i] = byteString.charCodeAt(i);
    }

    return new Blob([dataView], {type: mimeString});
  }

  function getJSON(url, successHandler, errorHandler) {
    var xhr = typeof XMLHttpRequest != 'undefined'
      ? new XMLHttpRequest()
      : new ActiveXObject('Microsoft.XMLHTTP');
      xhr.open('get', url, true);
      xhr.onreadystatechange = function() {
        var status;
        var data;
        if (xhr.readyState == 4) { // `DONE`
          status = xhr.status;
          if (status == 200 || status == 0) {
            data = JSON.parse(xhr.responseText);
            successHandler && successHandler(data);
          } else {
            errorHandler && errorHandler(status);
          }
        }
      };
      xhr.send();
  }

  window.Halftone = UI;

}();
