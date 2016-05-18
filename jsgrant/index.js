'use strict'

const util = require("util")

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
      cb(null, false, '"' + me + '" has insufficient priviledges to authorize "'
                     + user + '" access to "' + resource + '"')
      return
    }

    const usersForResource = resources[resource]
    if (!usersForResource)
    {
      cb(null, false, 'Resource "' + resource + '" does not exist')
      return
    }

    let current = usersForResource.find ((userInfo) => {
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

      cb(null, true, '"' + user + '" now has "' + readOnlyTxt + '" access to "' + resource +'"')
      return
    }

  })

}

function revokePermission (me, user, resource, readOnly, cb) {

  isAuthorized(me, resource, readOnly, (err, authorized) => {
    if(err) {
      cb('insuffisent priviledge')
      return
    }

    const index = resources[resource].findIndex( (userInfo) => {
      return userInfo.username == user
    })
    if (index > -1) {
      resources[resource].splice(index, 1)
    }
    // success, even if the permission wasn't there
    // tbd write to db
    cb(null)

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
  console.log('           RESOURCES '+  util.inspect(resources))
  console.log('body '+  util.inspect(req.body))
  console.log('query ' + util.inspect(req.query))
  const token = req.query.granterToken
  const granter = token
  const grantee  = req.query.grantee
  const resource = req.query.resource
  const readOnly = JSON.parse(req.query.readOnly)

  grantPermission(granter, grantee, resource, readOnly, (err, success, msg)=>{
    console.log("CB grant permission cb" , err, success, msg)

     if (err) {
        res.jsonp({success: false, msg: err})
        return
     }
     res.jsonp({success: success, msg: msg})
     return
  })
}

// route for revoke
function revoke(req, res) {
  let s = `{revoked: false}`
  console.log('revoke ' + s)
  res.jsonp(s)
}




exports.init = init
exports.grant = grant
exports.revoke = revoke

