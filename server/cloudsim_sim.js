'use strict'

const express = require('express')
const app = express()
const fs = require('fs')
const bodyParser = require("body-parser")
const cors = require('cors')
const morgan = require('morgan')
const dotenv = require('dotenv')
// cloudsim module
const csgrant = require('cloudsim-grant')
// local modules
const simulations = require('./simulations')
const downloads = require('./downloads')
const websocket = require('./websocket')

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

// The Cross-Origin Resource Sharing standard
app.use(cors())
// Populates the body of each request
app.use(bodyParser.json())

// prints all requests to the terminal
app.use(morgan('combined'))

// the port of the server
const port = process.env.PORT || 4000
// the delay between simulator process state machine update in ms
const simulationsSchedulerInterval = process.env.SCHEDULER_INTERVAL || 1000

process.env.CLOUDSIM_ADMIN = process.env.CLOUDSIM_ADMIN || 'admin'

// setup
// error if file is not there
const pathToKeysFile = __dirname + '/keys.zip'
console.log('path to keys: ' + pathToKeysFile)
fs.statSync(pathToKeysFile)

const dbName = 'cloudsim-sim' + (process.env.NODE_ENV == 'test'? '-test': '')
csgrant.showLog = true

// we create 2 initial resources
csgrant.init(process.env.CLOUDSIM_ADMIN, {'simulations': {},
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

console.log('admin user: ' + process.env.CLOUDSIM_ADMIN)
console.log ('database: ' + dbName)

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
app.get('/permissions',
    csgrant.authenticate,
    csgrant.userResources,
    csgrant.allResources
)

// get user permissions for a resource
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

// local modules have routes too
simulations.setRoutes(app)
downloads.setRoutes(app)

// shutdown nicely
process.on('SIGTERM', ()=>{
  console.log('SIGTERM')
  simulations.stopSimulationScheduler()
})

// make csgrant available to tests
app.csgrant = csgrant
// Expose app
exports = module.exports = app;

// start the server
httpServer.listen(port, function(){
  console.log('ssl: ' + useHttps)
	console.log('listening on *:' + port);
})
