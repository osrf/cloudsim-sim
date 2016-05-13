'use strict'


exports.desc = function() {
 console.log('hey la')
}

let resources = {launch : [] }

exports.grantPermission = function(me, user, permission, resource, readOnly)
{
  if(!isAuthorized(me, permission, resource, readOnly) ) {
    throw new Error('insuffisent priviledge')
  }

  if(isAuthorized(user, permission, resource, readOnly) ) {
    // already authorized.
    return
  }

  let x = { user : user,
            readOnly : readOnly,
            permission : permission,
            resource : resource,
            authority : me
           }

  resources[resource].permissions.append(x)
}

function revokePermission (me, user, permission, resource, readOnly) {
  if(!isAuthorized(me, permission, resource, readOnly) ) {
    throw new Error('insuffisent priviledge')
  }
  const index = resources[resource].permissions.findIndex( (userInfo) => {
      return userInfo.username == user
    })
  if (index > -1) {
    resources[resource].permissions.splice(index, 1)
  }
}

function isAuthorized(user, permission, resource, readOnly) {
  const users = getAllowedUsers(permission, resource)
  const info = users.find ( (userInfo) => {
      return userInfo.username == user
    } )

  if (!info || info.readonly != readOnly)
    return false
  return true
}

function getAllowedUsers (permission, resource, readOnly) {
  return []
}

function loadPermissions() {
 resources = {}
}

exports.loadPermissions = loadPermissions
exports.revokePermission = revokePermission
exports.isAuthorized = isAuthorized
exports.getAllowedUsers = getAllowedUsers


