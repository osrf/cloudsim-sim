'use strict'



let resources = {launch : [] }
let pubkey = ''
let authServerIp = ''

function init(ip, key, user ) {
  authServerIp = ip
  pubkey = key
  resources.launch = [{username: user}]
  loadPermissions( () =>{
    console.log('read all grants')
  })
}

function loadPermissions(cb) {
  resources = {}
  cb(null, resources)
}


function grantPermission(me, user, resource, readOnly, cb) {

  isAuthorized(me, resource, readOnly, (err, authorized) =>  {
    if (err) {
      cb(err)
      return
    }

    if (!authorized) {
      cb('insuffisent priviledge')
      return
    }

    // check if already authorized
    isAuthorized(user, permission, resource, readOnly, (err, authorized)=> {

      if (err) {
        cb(err)
        return
      }
      if (authorized) {// already authorized.
        cb(null)
        return
      }
      let x = { user : user,
            readOnly : readOnly,
            resource : resource,
            authority : me
           }
      resources[resource].permissions.append(x)
      console.log('to DB: ' + x)
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

    const index = resources[resource].permissions.findIndex( (userInfo) => {
      return userInfo.username == user
    })
    if (index > -1) {
      resources[resource].permissions.splice(index, 1)
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
    if (!info || info.readonly != readOnly) {
      // not authorized
      cb(null, false)
      return
    }

    cb(null, true)

  })
}

function getAllowedUsers (resource, readOnly, cb) {
  console.log('getAllowedUsers ' + resource)
  let users = resources[resource].permissions
  cb(null, users)
}


exports.init = init
exports.grantPermission = grantPermission
exports.revokePermission = revokePermission
exports.isAuthorized = isAuthorized
exports.getAllowedUsers = getAllowedUsers


