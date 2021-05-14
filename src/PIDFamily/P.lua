--!strict

--[[
    Copyright 2021 - Zach Curtis
    Proportional Controller

    See: https://www.csimn.com/CSI_pages/PIDforDummies.html
]]

---@class PController
local P = {}
P.__index = P

---@param kP number @proportional gain
---@param sP number @setpoint variable
---@return PController
function P.new(kP, sP)
    
    local self = setmetatable({}, P)

    self.ProportionalGain = kP
    self.Setpoint = sP or 0

    return self
end

---@param sP number @setpoint variable 
function P:UpdateSetpoint(sP)
    self.Setpoint = sP
end

-- call in a loop
---@param pV number @process variable 
---@param injectedError number @optional externally injected error to use instead of calculating error from stored SetPoint
---@return number @sensor feedback value
function P:Step(pV, injectedError)
    local err = injectedError and injectedError or self.Setpoint - pV

    return err * self.ProportionalGain
end

return P