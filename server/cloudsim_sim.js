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

// the configuration values are set in the local .env file
// this loads the .env content and puts it in the process environment.
dotenv.load()

let httpServer = null

const useHttps = false
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

const adminUser = process.env.CLOUDSIM_ADMIN || 'admin'

// setup
// error if file is not there
const pathToKeysFile = __dirname + '/keys.zip'
fs.statSync(pathToKeysFile)

const dbName = 'cloudsim-sim' + (process.env.NODE_ENV == 'test'? '-test': '')


// gets info that is printed in the console, as well as /
function details() {
  const date = new Date()
  const version = require('../package.json').version
  const csgrantVersion = require('cloudsim-grant/package.json').version
  const env = app.get('env')

  const s = `
date: ${date}
cloudsim-sim version: ${version}
port: ${port}
cloudsim-grant version: ${csgrantVersion}
admin user: ${adminUser}
environment: ${env}
redis database name: ${dbName}
redis database url: localhost
path to keys: ${pathToKeysFile}
`
  return s
}

// write details to the console
console.log('============================================')
console.log(details())
console.log('============================================')

app.get('/', function (req, res) {
  const info = details()
  const s = `
    <h1>Cloudsim-sim server</h1>
    <div>Gazebo controller is running</div>
    <pre>
  	${info}
    </pre>
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

// we create 2 initial resources,
// load the database and start serving
// when ready
csgrant.init(adminUser,
  { 'simulations': {},
   'downloads': {path: pathToKeysFile}
  },
  dbName,
  'localhost',
  httpServer,
  (err)=> {
    if(err) {
      console.log('Error loading resources: ' + err)
      process.exit(-2)
    }
    else {
      console.log('resources loaded')
      // Run the simulator process scheduler
      simulations.startSimulationsScheduler(simulationsSchedulerInterval)

      // start the server
      httpServer.listen(port, function(){
        console.log('ssl: ' + useHttps)
        console.log('listening on *:' + port);
      })
    }
  }
)

