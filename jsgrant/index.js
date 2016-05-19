'use strict'

const util = require("util")
const jstoken = require("./token")

let resources = {gzserver : [] }
let pubkey = ''
let authServerIp = ''

// ip: the authentication ip to verify that users exist
// key: the public authentication server key, to verify tokens
// user: the 'admin' original user
function init(ip, key, user ) {
  console.log('init jsgrant: ', ip, key, user)
  authServerIp = ip
  pubkey = key
  resources.gzserver = [{username: user, readOnly: false}]
  loadPermissions( () =>{
    console.log('read all grants')
  })
}

function loadPermissions(cb) {
  //resources = {}
  cb(null, resources)
}


function grantPermission(me, user, resource, readOnly, cb) {
  console.log('TODO: WRITE TO DB: grant', me, ' => ', user, resource, readOnly)

  // Am I authorized to grant this permission
  isAuthorized(me, resource, readOnly, (err, authorized) =>  {

    // Error getting my authorization
    if (err) {
      cb(err)
      return
    }

    // I'm not authorized to give this permission
    if (!authorized) {
      cb(null, false, '"' + me + '" has insufficient priviledges to manage "'
                     + user + '" access for "' + resource + '"')
      return
    }

    const resourceUsers = resources[resource]
    if (!resourceUsers)
    {
      cb(null, false, 'Resource "' + resource + '" does not exist')
      return
    }

    let current = resourceUsers.find ((userInfo) => {
      return userInfo.username == user
    })
    // If user already has some authorization
    if (current)
    {
      // Is already read only
      if ((readOnly == true) && (current.readOnly == true))
      {
        cb(null, true, '"' + user + '" is already authorized for "read only" for "'
           + resource + '"')
        return
      }
      // Is already write
      if ((readOnly == false) && (current.readOnly == false))
      {
        cb(null, true, '"' + user + '" is already authorized for "write" for "'
           + resource + '"')
        return
      }
      // Is write and we want to downgrade
      if ((readOnly == true) && (current.readOnly == false))
      {
        current.readOnly = true
        cb(null, true, '"' + user + '" access for "'
           + resource + '" has been downgraded to "read only"')
        return
      }
      // Is read only and we want to upgrade
      if ((readOnly == false) && (current.readOnly == true))
      {
        current.readOnly = false
        cb(null, true, '"' + user + '" access for "'
           + resource + '" has been upgraded to "write"')
        return
      }

      cb("Something went wrong")
      return;
    }
    else
    {
      // Grant brand new permission
      let x = { username : user,
                readOnly : readOnly,
                resource : resource,
                authority : me
              }
      resources[resource].push(x)

      var readOnlyTxt = readOnly? "read only" : "write"

      cb(null, true, '"' + user + '" now has "' + readOnlyTxt + '" access for "' + resource +'"')
      return
    }

  })

}

function revokePermission (me, user, resource, readOnly, cb) {

  console.log('TODO: WRITE TO DB: revoke', me, ' => ', user, resource, readOnly)

  // Am I authorized to revoke this permission
  isAuthorized(me, resource, readOnly, (err, authorized) =>  {

    // Error getting my authorization
    if (err) {
      cb(err)
      return
    }

    // I'm not authorized to give this permission
    if (!authorized) {
      cb(null, false, '"' + me + '" has insufficient priviledges to manage "'
                     + user + '" access for "' + resource + '"')
      return
    }

    const resourceUsers = resources[resource]
    if (!resourceUsers)
    {
      cb(null, false, 'Resource "' + resource + '" does not exist')
      return
    }

    let current = resourceUsers.find ((userInfo) => {
      return userInfo.username == user
    })
    // If user has no authorization
    if (!current)
    {
      cb(null, true, '"' + user + '" has no authorization for "' + resource + '" so nothing changed.')
      return
    }
    else
    {
      // Is read only, revoking read only
      if ((readOnly == true) && (current.readOnly == true))
      {
        resourceUsers.splice(resourceUsers.indexOf(current), 1)
        cb(null, true, '"' + user + '" is no longer authorized for "read only" for "'
           + resource + '"')
        return
      }
      // Is write, revoking write
      if ((readOnly == false) && (current.readOnly == false))
      {
        resourceUsers.splice(resourceUsers.indexOf(current), 1)
        cb(null, true, '"' + user + '" is no longer authorized for "write" for "'
           + resource + '"')
        return
      }
      // Is write and we want to revoke read-only - not allowed
      if ((readOnly == true) && (current.readOnly == false))
      {
        cb(null, false, '"' + user + '" has "write" access for "'
           + resource + '", so "read only" can\'t be revoked.')
        return
      }
      // Is read-only and want to revoke write - remove it all
      if ((readOnly == false) && (current.readOnly == true))
      {
        resourceUsers.splice(resourceUsers.indexOf(current), 1)
        cb(null, true, '"' + user + '" had "read only" access for "'
           + resource + '" and now has nothing')
        return
      }

      cb("Bad bad widget, something went wrong")
      return;
    }

  })

}

// Check if a user already has a given permission for a resource
function isAuthorized(user, resource, readOnly, cb) {
  getResourceUsers(resource, function (err, users) {

    // Error getting users for this resource
    if (err) {
      cb(err)
      return
    }

    // Get user's current permission for this resource
    const current = users.find ((userInfo) => {
        return userInfo.username == user
    })

    // User not authorized for this resource yet
    if (!current) {
      console.log('User not authorized yet: ', user, resource, readOnly)
      cb(null, false)
      return
    }

    // Currently read only, so not write authorized
    if(current.readonly && readOnly == false) {
      // not authorized
      console.log('read only not authorized: ', user, resource, readOnly)
      cb(null, false)
      return
    }
    // the user has access
    cb(null, true)

  })
}

// Returns all users for the given resource
// resource: resource to check
// cb: callback with params:
// - error
// - list of users
function getResourceUsers (resource, cb) {
  let users = resources[resource]

  // resource does not exist, therefore, no users
  if (!users)
    users = []

  cb(null, users)
}


// route for grant
function grant(req, res) {
  console.log('Grant, query: ' + util.inspect(req.query))
  const token = req.query.granterToken
  const grantee  = req.query.grantee
  const resource = req.query.resource
  const readOnly = JSON.parse(req.query.readOnly)

  console.log('  token: ' + token
  jstoken.verifyToken (token, (decoded) => {
    console.log('  decoded token: ' + JSON.stringify(decoded))
    const granter = decoded.username
    grantPermission(granter, grantee, resource, readOnly, (err, success, message)=>{
       success = true
       let msg = message
       if (err) {
          success = false
          msg =  err
       }

       const r ={   operation: 'grant',
                    granter: granter,
                    grantee: grantee,
                    resource: resource,
                    readOnly: readOnly,
                    success: success,
                    msg: msg
                 }
       res.jsonp(r)
    })
  })
}


// route for revoke
function revoke(req, res) {
  console.log('Revoke, query: ' + util.inspect(req.query))
  const token = req.query.granterToken
  const grantee  = req.query.grantee
  const resource = req.query.resource
  const readOnly = JSON.parse(req.query.readOnly)

  jstoken.verifyToken (token, (decoded) => {
    console.log('  decoded token: ' + JSON.stringify(decoded))
    const granter = decoded.username
    revokePermission(granter, grantee, resource, readOnly, (err, success, message)=>{
       success = true
       let msg = message
       if (err) {
          success = false
          msg = err
       }

       const r ={   operation: 'revoke',
                    granter: granter,
                    grantee: grantee,
                    resource: resource,
                    readOnly: readOnly,
                    success: success,
                    msg: msg
                 }
       res.jsonp(r)
    })
  })
}




exports.init = init
exports.grant = grant
exports.revoke = revoke

exports.signToken = jstoken.signToken
exports.verifyToken = jstoken.verifyToken
exports.test = jstoken.test
