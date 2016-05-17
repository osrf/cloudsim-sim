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
  console.log('grant permission', me, ' => ', user, resource, readOnly)
  isAuthorized(me, resource, readOnly, (err, authorized) =>  {
    if (err) {
      console.log('error ' + err)
      cb(err)
      return
    }

    if (!authorized) {
      console.log ( '"' + me + '" has insufficient priviledges to authorize "'
                     + user + '" access to "' + resource + '"')
      cb('insuffisent priviledge')
      return
    }

    // check if already authorized
    isAuthorized(user, resource, readOnly, (err, authorized)=> {

      if (err) {
        console.log('error: ' + err)
        cb(err)
        return
      }
      if (authorized) {// already authorized.
        console.log('"' + user + '" is already authorized for "'
           + resource + '", readOnly: ' + readOnly)
        cb(null)
        return
      }
      let x = { user : user,
            readOnly : readOnly,
            resource : resource,
            authority : me
           }
      resources[resource].push(x)
      console.log('todo DB: ' + x)
      cb(null)

    })

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

function isAuthorized(user, resource, readOnly, cb) {
  getAllowedUsers(resource, readOnly, function (err, users) {

    if (err) {
      cb(err)
      return
    }
    // is
    const info = users.find ( (userInfo) => {
        return userInfo.username == user
    })
    // user not in the list
    if (!info) {
      console.log('User not on the list : ', user, resource, readOnly)
      cb(null, false)
      return
    }

    if(info.readonly && (readOnly == false) ) {
      // not authorized
      console.log('read only not authorized: ', user, resource, readOnly)
      cb(null, false)
      return
    }
    // the user has access
    cb(null, true)

  })
}

function getAllowedUsers (resource, readOnly, cb) {
  console.log('getAllowedUsers ' + resource)
  let users = resources[resource]
  // resource does not exists, therefore, no users
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
  const readOnly = req.query.readOnly
  grantPermission(granter, grantee, resource, readOnly, (err, authorized)=>{

     if (err) {
        res.jsonp({granted: false, error: err})
        return
     }
     res.jsonp({granted: authorized})
  })

  // res.sendFile(__dirname + '/../public/index.html')
  let s = {granted: false}
  console.log('grant: ' + s)
  res.jsonp(s)
}

// route for revoke
function revoke(req, res) {
  let s = `
    {
      revoked: false
    }
`
  console.log('revoke ' + s)
  res.jsonp(s)
}




exports.init = init
exports.grant = grant
exports.revoke = revoke

/*
exports.grantPermission = grantPermission
exports.revokePermission = revokePermission
exports.isAuthorized = isAuthorized
exports.getAllowedUsers = getAllowedUsers
*/

