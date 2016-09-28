'use strict'

const ansi_to_html = require('ansi-to-html')
const socketioJwt = require('socketio-jwt');
const UnauthorizedError = require('./UnauthorizedError');
const jwt = require('jsonwebtoken');
const ansi2html = new ansi_to_html()

function autho(socket) {
  console.log('\n\nautho for new socket')

  var options = {
    // secret: Buffer(JSON.stringify(process.env.CLIENT_SECRET), 'base64'),
    secret: Buffer(JSON.stringify("gzsecret"), 'base64'),
    timeout: 15000 // 15 seconds to send the authentication message
  }

  var server = this.server || socket.server;

  if (!server.$emit) {
    //then is socket.io 1.0
    var Namespace = Object.getPrototypeOf(server.sockets).constructor;
    if (!~Namespace.events.indexOf('authenticated')) {
      Namespace.events.push('authenticated');
    }
  }
  if(options.required){
    var auth_timeout = setTimeout(function () {
      socket.disconnect('unauthorized');
    }, options.timeout || 5000);
  }

  socket.on('authenticate', function (data) {
    console.log('authenticate... token[' + data.token + ']')
    if(options.required){
      clearTimeout(auth_timeout);
    }
    // error handler
    var onError = function(err, code) {
        if (err) {
          code = code || 'unknown';
          var error = new UnauthorizedError(code, {
            message: (Object.prototype.toString.call(err) === '[object Object]' && err.message) ? err.message : err
          });
          socket.emit('unauthorized', error, function() {
            socket.disconnect('unauthorized');
          });
          return; // stop logic, socket will be close on next tick
        }
    };
    if(typeof data.token !== "string") {
      return onError({message: 'invalid token datatype'}, 'invalid_token');
    }

    var onJwtVerificationReady = function(err, decoded) {
      console.log('after token verification')
      // success handler
      var onSuccess = function() {
        console.log('on success decoded: ' + JSON.stringify(decoded))
        socket.decoded_token = decoded;
        socket.emit('authenticated');
        if (server.$emit) {
          server.$emit('authenticated', socket);
        } else {
          //try getting the current namespace otherwise fallback to all sockets.
          var namespace = (server.nsps && socket.nsp &&
                           server.nsps[socket.nsp.name]) ||
                          server.sockets;

          // explicit namespace
          namespace.emit('authenticated', socket);
        }
      };

      console.log('BYPASS!!')
      return onSuccess()

      if (err) {
        console.log('error during validation');
        return onError(err, 'invalid_token');
      }

      if(options.additional_auth &&
          typeof options.additional_auth === 'function') {
        options.additional_auth(decoded, onSuccess, onError)
      } else {
        onSuccess()
      }
    }

    var onSecretReady = function(err, secret) {
      if (err || !secret) {
        return onError(err, 'invalid_secret');
      }
      console.log('secret: ' + secret)
      jwt.verify(data.token, secret, options, onJwtVerificationReady);

    };

    // console.log('call getSecret from socket.on("authenticate")')
    // getSecret(socket.request, options.secret, data.token, onSecretReady);
    console.log('onSecretReady')
    onSecretReady(null, options.secret)
  })
}

// setup websocket to control processes
function init(httpServer) {
  const io = require('socket.io')(httpServer)
  io
    .on('connection', autho )
    .on('authenticated', function(socket){
      console.log('connected & authenticated: ' + JSON.stringify(socket.decoded_token));

      let gzcmd = {proc:null, output:'', state: 'ready', cmdline:''}

      socket.on('gz-cmd', function(msg) {
        console.log('received: ' + JSON.stringify(msg))
        if (msg.cmd === 'run'){
          const items = msg.cmdline.split(' ')
          const proc = items[0]
          const args = items.slice(1)
          console.log('spwaning: ' + proc + ' ' + args)

          gzcmd.proc = spawn(proc, args, {stdio:'pipe'})
          gzcmd.state = 'running'
          gzcmd.cmdline = msg.cmdline

          var onNewData = function (buf) {
            const txt = buf.toString()
            // replace new lines with html line breaks
            const html = txt.split('\n').join('<br>')
            // convert the console color codes to html
            //   ex: "[0m[1;31m:[0m[1;31m96[0m[1;31m] [0m[1;31m"
            const ansi = ansi2html.toHtml(html)
            gzcmd.output += ansi
            const msg = {output: gzcmd.output,
              state:gzcmd.state,
              pid:gzcmd.proc.pid }
            console.log(msg)
            return msg
          }

          gzcmd.proc.stdout.on('data', (data)=> {
            io.emit('gz-cmd', onNewData(data))
          })
          gzcmd.proc.stderr.on('data', (data)=> {
            io.emit('gz-cmd', onNewData(data))
          })
          gzcmd.proc.on('close', (code)=>{
            console.log('gzcmd.proc.on close')
            gzcmd.state = 'closed'
            // tell client
            io.emit('gz-cmd', {output: gzcmd.output,
              state:gzcmd.state,
              pid:gzcmd.proc.pid })
            gzcmd = null
          })
        }
        if (msg.cmd === 'kill'){
          console.log('kill message received')
          gzcmd.proc.kill()
        }
      })
    })
  return io
}

exports.init = init
