'use strict'

const express = require('express')
const app = express()
const fs = require('fs')
const bodyParser = require("body-parser")
const cors = require('cors')
const morgan = require('morgan')
const util = require('util')
const dotenv = require('dotenv')
const xtend = require('xtend');
const spawn = require('child_process').spawn
const csgrant = require('cloudsim-grant')

const simulations = require('./simulations')
const downloads = require('./downloads')
const websocket = require('./websocket')

//const ansi_to_html = require('ansi-to-html')
//const socketioJwt = require('socketio-jwt');
//const UnauthorizedError = require('./UnauthorizedError');
//const jwt = require('jsonwebtoken');
//const ansi2html = new ansi_to_html()


// the configuration values are set in the local .env file
// this loads the .env content and puts it in the process environment.
dotenv.load()

let httpServer = null

const useHttps = true
if(useHttps) {
  const privateKey  = fs.readFileSync(__dirname + '/key.pem', 'utf8')
  const certificate = fs.readFileSync(__dirname + '/key-cert.pem', 'utf8')
  httpServer = require('https').Server({
    key: privateKey, cert: certificate
  }, app)
}
else {
  httpServer = require('http').Server(app)
}

const io = websocket.init(httpServer)

app.use(cors())
app.use(bodyParser.json())

// prints all requests to the terminal
app.use(morgan('combined'))

// the port of the server
const port = process.env.CLOUDSIM_PORT || 4000
// the delay between simulator process state machine update in ms
const simulationsSchedulerInterval = process.env.SCHEDULER_INTERVAL || 1000

process.env.ADMIN_USER = process.env.ADMIN_USER || 'admin'
const pathToKeysFile = __dirname + '/keys.zip'
console.log('path to keys: ' + pathToKeysFile)
// error if file is not there
fs.statSync(pathToKeysFile)

const dbName = 'cloudsim-sim' + process.env.NODE_ENV == 'test'? 'test': ''
// we create 2 initial resources
csgrant.init(process.env.ADMIN_USER, {'simulations': {},
                                      'downloads': {path: pathToKeysFile}
                                     },
                                     dbName,
                                     'localhost',
                                     (err)=> {
    if(err) {
      console.log('Error loading resources: ' + err)
      process.exit(-2)
    }
    else {
      console.log('resources loaded')
      // Run the simulator process scheduler
      simulations.startSimulationsScheduler(simulationsSchedulerInterval)
    }
})

console.log('admin user: ' + process.env.ADMIN_USER)
console.log ('database: ' + dbName)
/*
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
          options.additional_auth(decoded, onSuccess, onError);
        } else {
          onSuccess();
        }
      };

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
    });
  }

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

// app.use(express.static(__dirname + '../public'));
*/

app.get('/', function (req, res) {
  // res.sendFile(__dirname + '/../public/index.html')
  let s = `
    <h1>Cloudsim-sim server</h1>
    Gazebo controller is running
  `
  res.end(s)
})

// setup the routes

// grant user permission to a resource
// (add user to a group)
app.post('/permissions',
    csgrant.authenticate,
    csgrant.grant)

// revoke user permission
// (delete user from a group)
app.delete('/permissions',
    csgrant.authenticate,
    csgrant.revoke)

// get all user permissions for all resources
// (get all users for all groups)
app.get('/permissions',
    csgrant.authenticate,
    csgrant.ownsResource('simulations', true),
    csgrant.userResources,
    csgrant.allResources
)

// get user permissions for a resource
// (get users in a group)
app.get('/permissions/:resourceId',
    csgrant.authenticate,
    csgrant.ownsResource(':resourceId', true),
    csgrant.resource
)

/// param for resource name
app.param('resourceId', function(req, res, next, id) {
  req.resourceId = id
  next()
})



simulations.setRoutes(app)
downloads.setRoutes(app)

// Expose app
exports = module.exports = app;

httpServer.listen(port, function(){
  console.log('ssl: ' + useHttps)
	console.log('listening on *:' + port);
})

process.on('SIGTERM', ()=>{
  console.log('SIGTERM')
  simulations.stopSimulationScheduler()
})




