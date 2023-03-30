--!strict

--[[
    Copyright (c) 2023, Zach Curtis
    Proportional Intergral Derivative Controller

    See: https://csimn.com/CSI_pages/PID.html
]]

local PID = {}
PID.__index = PID

---@param kP number proportional gain - counters current error
---@param kI number intergral gain - counters accumulated error
---@param kD number derivative gain - counters oscillation
---@param iMax number maximum intergral value 
---@param sP number setpoint variable 
function PID.new(kP: number, kI: number, kD: number, iMax: number, sP: number)
    local self = setmetatable({}, PID)

    self.ProportionalGain = kP
    self.IntergralGain = kI
    self.DerivativeGain = kD

    self.IntergralMax = iMax

    self.Setpoint = sP

    self.IntergralTerm = 0
    self.LastError = 0

    return self
end

---@param sP number setpoint variable 
function PID:UpdateSetpoint(sP: number)
    self.Setpoint = sP
end

-- call in a loop
---@param pV number process variable
---@param dt number delta time (duration between last call to Step and this call to Step)
function PID:Step(pV: number, dt: number, returnAlpha: number)
    
    local err = self.Setpoint - pV

    local derivativeTerm = (err - self.LastError) / dt
    self.LastError = err
    
    self.IntergralTerm += err * dt

    -- clamp intergral in bounds
    if math.abs(self.IntergralTerm) > self.IntergralMax then
        local i = self.IntergralTerm > 0 and 1 or -1

        self.IntergralTerm = i * self.IntergralMax
    end

    local ProportionalTerm = err * self.ProportionalGain
    local IntergralTerm = self.IntergralTerm * self.IntergralGain
    local DerivativeTerm = derivativeTerm * self.DerivativeGain

    if returnAlpha then
        return math.clamp(ProportionalTerm + IntergralTerm + DerivativeTerm, 0, 1)
    else
        return ProportionalTerm + IntergralTerm + DerivativeTerm
    end
end

function PID:Reset()
    self.IntergralTerm = 0
end

return PID