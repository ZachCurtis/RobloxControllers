--!strict

--[[
    Copyright (c) 2021, Zach Curtis
    Proportional Intergral Controller

    See: https://www.csimn.com/CSI_pages/PIDforDummies.html
]]

local PI = {}
PI.__index = PI

---@param kP number proportional gain
---@param kI number intergral gain
---@param iMax number maximum intergral value
---@param sP number setpoint variable 
function PI.new(kP: number, kI: number, iMax: number, sP: number)
    local self = setmetatable({}, PI)

    self.ProportionalGain = kP
    self.IntergralGain = kI

    self.IntergralMax = iMax

    self.Setpoint = sP

    self.IntergralTerm = 0

    return self
end

---@param sP number setpoint variable 
function PI:UpdateSetpoint(sP: number)
    self.Setpoint = sP
end

-- call in a loop
---@param pV number process variable
---@param dt number delta time (duration between last call to Step and this call to Step)
function PI:Step(pV: number, dt: number)
    local err = self.Setpoint - pV
    
    self.IntergralTerm += err * dt

    -- clamp intergral in bounds
    if math.abs(self.IntergralTerm) > self.IntergralMax then
        local i = self.IntergralTerm > 0 and 1 or -1

        self.IntergralTerm = i * self.IntergralMax
    end
    
    local ProportionalTerm = err * self.ProportionalGain

    return ProportionalTerm + (self.IntergralTerm * self.IntergralGain)
end

function PI:Reset()
    self.IntergralTerm = 0
end

return PI