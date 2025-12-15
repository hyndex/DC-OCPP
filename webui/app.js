const state = {
  config: {},
  history: [],
  meta: { path: '', modified: null, size: null },
  dirty: false,
  ocppText: '',
  lastError: '',
};

const sections = {};
let unloadBound = false;

function esc(value) {
  if (value === null || value === undefined) return '';
  return String(value)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#39;');
}

function setByPath(target, path, value) {
  const parts = path.split('.');
  let node = target;
  parts.forEach((p, idx) => {
    const isLast = idx === parts.length - 1;
    const key = Number.isInteger(Number(p)) && p.trim() !== '' ? Number(p) : p;
    if (isLast) {
      node[key] = value;
      return;
    }
    if (node[key] === undefined || typeof node[key] !== 'object') {
      node[key] = Number.isInteger(Number(parts[idx + 1])) ? [] : {};
    }
    node = node[key];
  });
}

function getByPath(target, path, fallback = '') {
  const parts = path.split('.');
  let node = target;
  for (const p of parts) {
    const key = Number.isInteger(Number(p)) && p.trim() !== '' ? Number(p) : p;
    if (node && Object.prototype.hasOwnProperty.call(node, key)) {
      node = node[key];
    } else {
      return fallback;
    }
  }
  return node;
}

function setDirty(flag = true) {
  state.dirty = flag;
  document.body.classList.toggle('dirty', flag);
  const badge = document.getElementById('dirty-indicator');
  if (badge) {
    badge.textContent = flag ? 'Unsaved changes' : 'Synced';
    badge.className = flag ? 'pill warning' : 'pill success';
  }
}

function toast(message, tone = 'neutral') {
  const el = document.getElementById('toast');
  if (!el) return;
  el.textContent = message;
  el.className = `toast ${tone}`;
  el.classList.add('show');
  setTimeout(() => el.classList.remove('show'), 2600);
}

function humanBytes(bytes) {
  if (!bytes || Number(bytes) === 0) return '0 B';
  const units = ['B', 'KB', 'MB', 'GB'];
  const idx = Math.floor(Math.log(bytes) / Math.log(1024));
  return `${(bytes / 1024 ** idx).toFixed(1)} ${units[idx]}`;
}

function humanTime(modified) {
  if (!modified) return '';
  const date = new Date(Number(modified));
  return date.toLocaleString();
}

async function fetchJSON(url, options = {}) {
  try {
    const res = await fetch(url, options);
    const data = await res.json();
    return { res, data };
  } catch (err) {
    throw new Error(`Network error: ${err.message || err}`);
  }
}

async function loadConfig() {
  const { res, data } = await fetchJSON('/api/config');
  if (!res.ok || !data.ok) {
    throw new Error(data.error || 'Unable to load config');
  }
  state.config = data.config || {};
  state.meta = { path: data.path, modified: data.modified, size: data.size };
  state.ocppText = JSON.stringify(state.config.ocpp || {}, null, 2);
  state.lastError = '';
  setDirty(false);
}

async function saveConfig() {
  const { res, data } = await fetchJSON('/api/config', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ config: state.config }),
  });
  if (!res.ok || !data.ok) {
    throw new Error(data.error || 'Save failed');
  }
  setDirty(false);
  await loadHistory();
  toast('Configuration saved', 'success');
  await loadConfig();
  renderAll();
}

async function validateConfig() {
  const { res, data } = await fetchJSON('/api/config/validate', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ config: state.config }),
  });
  if (!res.ok || !data.ok) {
    throw new Error(data.error || 'Validation failed');
  }
  toast(data.message || 'Config is valid', 'success');
}

async function backupConfig() {
  const { res, data } = await fetchJSON('/api/config/backup', { method: 'POST' });
  if (!res.ok || !data.ok) {
    throw new Error(data.error || 'Backup failed');
  }
  await loadHistory();
  renderRaw();
  toast('Backup created', 'success');
}

async function restoreBackup(path) {
  const { res, data } = await fetchJSON('/api/config/restore', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ path }),
  });
  if (!res.ok || !data.ok) {
    throw new Error(data.error || 'Restore failed');
  }
  await loadConfig();
  await loadHistory();
  renderAll();
  setDirty(false);
  toast('Restored backup', 'success');
}

async function loadHistory() {
  try {
    const { res, data } = await fetchJSON('/api/config/history');
    if (res.ok && data.ok) {
      state.history = data.history || [];
    }
  } catch (err) {
    state.history = [];
  }
}

function bindInputs(root) {
  root.querySelectorAll('[data-path]').forEach((el) => {
    const path = el.dataset.path;
    const type = el.dataset.type || el.type;
    const applyValue = () => {
      let value = el.value;
      if (type === 'number') {
        value = el.value === '' ? '' : Number(el.value);
      }
      if (type === 'checkbox' || type === 'bool') {
        value = el.checked;
      }
      setByPath(state.config, path, value);
      setDirty(true);
    };
    el.addEventListener('change', applyValue);
    el.addEventListener('input', applyValue);
  });
}

function navTo(section) {
  Object.values(sections).forEach((sec) => sec.classList.add('hidden'));
  Object.entries(sections).forEach(([key, sec]) => {
    const nav = document.querySelector(`[data-target="${key}"]`);
    if (nav) {
      nav.classList.toggle('active', key === section);
    }
  });
  if (sections[section]) {
    sections[section].classList.remove('hidden');
  }
}

function renderOverview() {
  const target = sections.overview;
  const cp = state.config.chargePoint || {};
  const connectors = state.config.connectors || [];
  const slots = state.config.slots || [];
  const ocpp = getByPath(state.config, 'ocpp.Internal', {});
  const moduleCount = slots.reduce((acc, slot) => acc + (slot.modules ? slot.modules.length : 0), 0);
  const cards = [
    { label: 'Charge Point ID', value: cp.id || '-', hint: ocpp.ChargePointId || '' },
    { label: 'Central System', value: cp.centralSystemURI || '-', hint: ocpp.CentralSystemURI || '' },
    { label: 'Connectors', value: connectors.length, hint: 'guns configured' },
    { label: 'Modules', value: moduleCount, hint: 'across slots' },
  ];
  target.innerHTML = `
    <div class="section-header">
      <div>
        <p class="eyebrow">Snapshot</p>
        <h2>Live picture of configs/charger.json</h2>
        <p class="muted">Edit fields in the side panels, validate, and push directly to disk.</p>
      </div>
      <div class="badge-row">
        <span class="pill">${state.meta.path || 'configs/charger.json'}</span>
        <span class="pill subtle">${humanBytes(state.meta.size)}</span>
        <span class="pill subtle">${humanTime(state.meta.modified)}</span>
      </div>
    </div>
    <div class="grid four">
      ${cards.map((c) => `
        <div class="card">
          <p class="muted">${esc(c.label)}</p>
          <p class="metric">${esc(c.value)}</p>
          <p class="muted small">${esc(c.hint || '')}</p>
        </div>
      `).join('')}
    </div>
    <div class="grid two stretch">
      <div class="card">
        <div class="card-head">
          <div>
            <p class="eyebrow">Charge Point Identity</p>
            <h3>${esc(cp.id || 'unset')}</h3>
          </div>
        </div>
        <div class="form-grid two">
          <label>Charge Point ID<input data-type="text" data-path="chargePoint.id" value="${esc(cp.id || '')}" /></label>
          <label>Central System URI<input data-type="text" data-path="chargePoint.centralSystemURI" value="${esc(cp.centralSystemURI || '')}" /></label>
          <label>Vendor<input data-type="text" data-path="chargePoint.vendor" value="${esc(cp.vendor || '')}" /></label>
          <label>Model<input data-type="text" data-path="chargePoint.model" value="${esc(cp.model || '')}" /></label>
          <label>Firmware<input data-type="text" data-path="chargePoint.firmwareVersion" value="${esc(cp.firmwareVersion || '')}" /></label>
          <label>CAN Interface<input data-type="text" data-path="chargePoint.canInterface" value="${esc(cp.canInterface || '')}" /></label>
        </div>
      </div>
      <div class="card">
        <div class="card-head">
          <div>
            <p class="eyebrow">PLC & Uploads</p>
            <h3>Safety + upload paths</h3>
          </div>
        </div>
        <div class="form-grid two">
          <label>Use PLC<input type="checkbox" data-type="bool" data-path="chargePoint.usePLC" ${cp.usePLC ? 'checked' : ''} /></label>
          <label>Use CRC8<input type="checkbox" data-type="bool" data-path="plc.useCRC8" ${getByPath(state.config, 'plc.useCRC8') ? 'checked' : ''} /></label>
          <label>Module Relays<input type="checkbox" data-type="bool" data-path="plc.moduleRelaysEnabled" ${getByPath(state.config, 'plc.moduleRelaysEnabled', true) ? 'checked' : ''} /></label>
          <label>HTTPS uploads<input type="checkbox" data-type="bool" data-path="plc.requireHttpsUploads" ${getByPath(state.config, 'plc.requireHttpsUploads', true) ? 'checked' : ''} /></label>
          <label>Upload cap (bytes)<input type="number" data-type="number" data-path="uploads.maxBytes" value="${esc(getByPath(state.config, 'uploads.maxBytes', ''))}" /></label>
          <label>Transfer timeout (s)<input type="number" data-type="number" data-path="uploads.transferTimeoutSeconds" value="${esc(getByPath(state.config, 'uploads.transferTimeoutSeconds', ''))}" /></label>
        </div>
      </div>
    </div>
  `;
  bindInputs(target);
}

function renderConnectors() {
  const target = sections.connectors;
  const connectors = state.config.connectors || [];
  setByPath(state.config, 'ocpp.Core.NumberOfConnectors', connectors.length || 0);
  target.innerHTML = `
    <div class="section-header">
      <div>
        <p class="eyebrow">Guns & meters</p>
        <h2>${connectors.length} connector${connectors.length === 1 ? '' : 's'} mapped</h2>
        <p class="muted">Edit limits and PLC ids. Remove connectors to shrink the charger footprint.</p>
      </div>
      <div class="actions">
        <button class="ghost" id="add-connector">Add connector</button>
      </div>
    </div>
    <div class="grid two stretch">
      ${connectors.map((c, idx) => `
        <div class="card connector-card" data-idx="${idx}">
          <div class="card-head">
            <div>
              <p class="eyebrow">Gun ${esc(c.id ?? idx + 1)}</p>
              <h3>${esc(c.label || 'Connector')}</h3>
            </div>
            <button class="ghost danger" data-remove="${idx}">Remove</button>
          </div>
          <div class="form-grid two">
            <label>Connector ID<input type="number" data-type="number" data-path="connectors.${idx}.id" value="${esc(c.id ?? idx + 1)}"/></label>
            <label>PLC ID<input type="number" data-type="number" data-path="connectors.${idx}.plcId" value="${esc(c.plcId ?? c.plc_id ?? 0)}"/></label>
            <label>Label<input data-path="connectors.${idx}.label" value="${esc(c.label || '')}" /></label>
            <label>CAN Interface<input data-path="connectors.${idx}.canInterface" value="${esc(c.canInterface || '')}" /></label>
            <label>Max Current (A)<input type="number" data-type="number" step="1" data-path="connectors.${idx}.maxCurrentA" value="${esc(c.maxCurrentA || '')}" /></label>
            <label>Max Power (W)<input type="number" data-type="number" step="1000" data-path="connectors.${idx}.maxPowerW" value="${esc(c.maxPowerW || '')}" /></label>
            <label>Max Voltage (V)<input type="number" data-type="number" step="10" data-path="connectors.${idx}.maxVoltageV" value="${esc(c.maxVoltageV || '')}" /></label>
            <label>Min Voltage (V)<input type="number" data-type="number" step="10" data-path="connectors.${idx}.minVoltageV" value="${esc(c.minVoltageV || '')}" /></label>
            <label>Meter Source<input data-path="connectors.${idx}.meterSource" value="${esc(c.meterSource || '')}" /></label>
            <label>Meter Scale<input type="number" data-type="number" step="0.01" data-path="connectors.${idx}.meterScale" value="${esc(c.meterScale || 1)}" /></label>
            <label>Meter Offset (Wh)<input type="number" data-type="number" step="1" data-path="connectors.${idx}.meterOffsetWh" value="${esc(c.meterOffsetWh || 0)}" /></label>
            <label>Meter Sample Interval (s)<input type="number" data-type="number" data-path="connectors.${idx}.meterSampleIntervalSeconds" value="${esc(c.meterSampleIntervalSeconds || state.config.meterSampleIntervalSeconds || '')}" /></label>
            <label>Lock Input<input type="number" data-type="number" data-path="connectors.${idx}.lockInputSwitch" value="${esc(c.lockInputSwitch || 3)}" /></label>
            <label class="checkbox">Require Lock<input type="checkbox" data-type="bool" data-path="connectors.${idx}.requireLock" ${c.requireLock !== false ? 'checked' : ''} /></label>
          </div>
        </div>
      `).join('')}
    </div>
  `;
  bindInputs(target);
  target.querySelectorAll('[data-remove]').forEach((btn) => {
    btn.addEventListener('click', () => {
      const idx = Number(btn.dataset.remove);
      state.config.connectors.splice(idx, 1);
      state.config.slots = (state.config.slots || []).filter((s) => s.gunId !== idx + 1);
      setDirty(true);
      renderConnectors();
      renderSlots();
      renderOverview();
      renderOcpp();
    });
  });
  const add = document.getElementById('add-connector');
  if (add) {
    add.onclick = () => {
      const nextId = ((state.config.connectors || []).length) + 1;
      state.config.connectors = state.config.connectors || [];
      state.config.connectors.push({
        id: nextId,
        plcId: nextId - 1,
        label: `Gun ${nextId}`,
        maxCurrentA: 200,
        maxPowerW: 60000,
        maxVoltageV: 1000,
        minVoltageV: 200,
        requireLock: true,
        lockInputSwitch: 3,
        meterSource: 'plc',
        meterScale: 1,
        meterOffsetWh: 0,
      });
      setDirty(true);
      renderConnectors();
      renderOverview();
      renderOcpp();
    };
  }
}

function renderSlots() {
  const target = sections.slots;
  const slots = state.config.slots || [];
  target.innerHTML = `
    <div class="section-header">
      <div>
        <p class="eyebrow">Slots & modules</p>
        <h2>${slots.length} slots mapped</h2>
        <p class="muted">Topology and module assignments across the ring.</p>
      </div>
      <div class="actions">
        <button class="ghost" id="add-slot">Add slot</button>
      </div>
    </div>
    <div class="grid one stretch">
      ${slots.map((slot, sIdx) => `
        <div class="card slot-card">
          <div class="card-head">
            <div>
              <p class="eyebrow">Slot ${esc(slot.id || sIdx + 1)}</p>
              <h3>Gun ${esc(slot.gunId || slot.id)}</h3>
            </div>
            <button class="ghost danger" data-remove-slot="${sIdx}">Remove</button>
          </div>
          <div class="form-grid four">
            <label>Slot ID<input type="number" data-type="number" data-path="slots.${sIdx}.id" value="${esc(slot.id || '')}" /></label>
            <label>Gun ID<input type="number" data-type="number" data-path="slots.${sIdx}.gunId" value="${esc(slot.gunId || '')}" /></label>
            <label>GC<input data-path="slots.${sIdx}.gc" value="${esc(slot.gc || '')}" /></label>
            <label>MC<input data-path="slots.${sIdx}.mc" value="${esc(slot.mc || '')}" /></label>
            <label>CW<input type="number" data-type="number" data-path="slots.${sIdx}.cw" value="${esc(slot.cw || '')}" /></label>
            <label>CCW<input type="number" data-type="number" data-path="slots.${sIdx}.ccw" value="${esc(slot.ccw || '')}" /></label>
            <label>Group<input data-path="slots.${sIdx}.group" value="${esc(slot.group || '')}" /></label>
          </div>
          <div class="modules">
            <div class="modules-head">
              <p class="eyebrow">${(slot.modules || []).length} modules</p>
              <button class="ghost" data-add-module="${sIdx}">Add module</button>
            </div>
            <div class="table">
              <div class="table-row header">
                <div>ID</div><div>MN</div><div>Type</div><div>Addr</div><div>Group</div><div>Power (kW)</div><div>Current (A)</div><div></div>
              </div>
              ${(slot.modules || []).map((m, mIdx) => `
                <div class="table-row">
                  <div><input data-path="slots.${sIdx}.modules.${mIdx}.id" value="${esc(m.id || '')}" /></div>
                  <div><input data-path="slots.${sIdx}.modules.${mIdx}.mn" value="${esc(m.mn || m.mn_id || '')}" /></div>
                  <div><input data-path="slots.${sIdx}.modules.${mIdx}.type" value="${esc(m.type || '')}" /></div>
                  <div><input type="number" data-type="number" data-path="slots.${sIdx}.modules.${mIdx}.address" value="${esc(m.address ?? '')}" /></div>
                  <div><input type="number" data-type="number" data-path="slots.${sIdx}.modules.${mIdx}.group" value="${esc(m.group ?? 0)}" /></div>
                  <div><input type="number" data-type="number" step="0.1" data-path="slots.${sIdx}.modules.${mIdx}.ratedPowerKW" value="${esc(m.ratedPowerKW || m.rated_power_kw || '')}" /></div>
                  <div><input type="number" data-type="number" step="0.1" data-path="slots.${sIdx}.modules.${mIdx}.ratedCurrentA" value="${esc(m.ratedCurrentA || m.rated_current_a || '')}" /></div>
                  <div><button class="ghost danger" data-remove-module="${sIdx}:${mIdx}">×</button></div>
                </div>
              `).join('')}
            </div>
          </div>
        </div>
      `).join('')}
    </div>
  `;
  bindInputs(target);
  target.querySelectorAll('[data-add-module]').forEach((btn) => {
    btn.addEventListener('click', () => {
      const sIdx = Number(btn.dataset.addModule);
      state.config.slots[sIdx].modules = state.config.slots[sIdx].modules || [];
      state.config.slots[sIdx].modules.push({
        id: `M${sIdx + 1}_${state.config.slots[sIdx].modules.length}`,
        mn: `MN_${sIdx + 1}_${state.config.slots[sIdx].modules.length}`,
        type: 'maxwell-mxr',
        address: state.config.slots[sIdx].modules.length,
        group: 0,
        ratedPowerKW: state.config.modulePowerKW || 30,
      });
      setDirty(true);
      renderSlots();
    });
  });
  target.querySelectorAll('[data-remove-module]').forEach((btn) => {
    btn.addEventListener('click', () => {
      const [sIdx, mIdx] = btn.dataset.removeModule.split(':').map(Number);
      state.config.slots[sIdx].modules.splice(mIdx, 1);
      setDirty(true);
      renderSlots();
    });
  });
  target.querySelectorAll('[data-remove-slot]').forEach((btn) => {
    btn.addEventListener('click', () => {
      const idx = Number(btn.dataset.removeSlot);
      state.config.slots.splice(idx, 1);
      setDirty(true);
      renderSlots();
    });
  });
  const addSlot = document.getElementById('add-slot');
  if (addSlot) {
    addSlot.onclick = () => {
      const nextId = ((state.config.slots || []).length) + 1;
      state.config.slots = state.config.slots || [];
      state.config.slots.push({
        id: nextId,
        gunId: nextId,
        gc: `GC_${nextId}`,
        mc: `MC_${nextId}`,
        cw: nextId + 1,
        ccw: nextId - 1 || 1,
        modules: [
          { id: `M${nextId}_0`, mn: `MN_${nextId}_0`, type: 'maxwell-mxr', address: (nextId - 1) * 2, group: 0, ratedPowerKW: state.config.modulePowerKW || 30 },
          { id: `M${nextId}_1`, mn: `MN_${nextId}_1`, type: 'maxwell-mxr', address: (nextId - 1) * 2 + 1, group: 0, ratedPowerKW: state.config.modulePowerKW || 30 },
        ],
      });
      setDirty(true);
      renderSlots();
      renderOverview();
    };
  }
}

function renderLimits() {
  const target = sections.limits;
  const planner = state.config.planner || {};
  const timeouts = state.config.timeouts || {};
  target.innerHTML = `
    <div class="section-header">
      <div>
        <p class="eyebrow">Limits & planner</p>
        <h2>Energy + timing envelopes</h2>
        <p class="muted">Grid headroom, module defaults, precharge, and timeouts.</p>
      </div>
    </div>
    <div class="grid two stretch">
      <div class="card">
        <div class="card-head">
          <h3>Power Limits</h3>
        </div>
        <div class="form-grid two">
          <label>Grid Limit (kW)<input type="number" data-type="number" step="10" data-path="gridLimitKW" value="${esc(state.config.gridLimitKW || '')}" /></label>
          <label>Module Power (kW)<input type="number" data-type="number" step="1" data-path="modulePowerKW" value="${esc(state.config.modulePowerKW || '')}" /></label>
          <label>Default Voltage (V)<input type="number" data-type="number" step="10" data-path="defaultVoltageV" value="${esc(state.config.defaultVoltageV || '')}" /></label>
          <label>Meter Sample Interval (s)<input type="number" data-type="number" data-path="meterSampleIntervalSeconds" value="${esc(state.config.meterSampleIntervalSeconds || '')}" /></label>
        </div>
      </div>
      <div class="card">
        <div class="card-head">
          <h3>Planner</h3>
        </div>
        <div class="form-grid two">
          <label>Max Modules / Gun<input type="number" data-type="number" data-path="planner.maxModulesPerGun" value="${esc(planner.maxModulesPerGun || '')}" /></label>
          <label>Min Modules Active<input type="number" data-type="number" data-path="planner.minModulesPerActiveGun" value="${esc(planner.minModulesPerActiveGun || '')}" /></label>
          <label>Max Island Radius<input type="number" data-type="number" data-path="planner.maxIslandRadius" value="${esc(planner.maxIslandRadius || '')}" /></label>
          <label>Allow Cross Slot<input type="checkbox" data-type="bool" data-path="planner.allowCrossSlotIslands" ${planner.allowCrossSlotIslands ? 'checked' : ''} /></label>
          <label>Precharge Tolerance (V)<input type="number" data-type="number" data-path="planner.prechargeVoltageToleranceV" value="${esc(planner.prechargeVoltageToleranceV || '')}" /></label>
          <label>Precharge Timeout (ms)<input type="number" data-type="number" data-path="planner.prechargeTimeoutMs" value="${esc(planner.prechargeTimeoutMs || '')}" /></label>
          <label>Min Module Hold (ms)<input type="number" data-type="number" data-path="planner.minModuleHoldMs" value="${esc(planner.minModuleHoldMs || '')}" /></label>
          <label>Min MC Hold (ms)<input type="number" data-type="number" data-path="planner.minMcHoldMs" value="${esc(planner.minMcHoldMs || '')}" /></label>
          <label>Min GC Hold (ms)<input type="number" data-type="number" data-path="planner.minGcHoldMs" value="${esc(planner.minGcHoldMs || '')}" /></label>
          <label>MC Open Current (A)<input type="number" data-type="number" data-path="planner.mcOpenCurrentA" value="${esc(planner.mcOpenCurrentA || '')}" /></label>
          <label>GC Open Current (A)<input type="number" data-type="number" data-path="planner.gcOpenCurrentA" value="${esc(planner.gcOpenCurrentA || '')}" /></label>
        </div>
      </div>
      <div class="card">
        <div class="card-head">
          <h3>Timeouts</h3>
        </div>
        <div class="form-grid two">
          <label>Authorization (s)<input type="number" data-type="number" data-path="timeouts.authorizationSeconds" value="${esc(timeouts.authorizationSeconds || '')}" /></label>
          <label>Power Request (s)<input type="number" data-type="number" data-path="timeouts.powerRequestSeconds" value="${esc(timeouts.powerRequestSeconds || '')}" /></label>
          <label>EVSE Limit Ack (ms)<input type="number" data-type="number" data-path="timeouts.evseLimitAckMs" value="${esc(timeouts.evseLimitAckMs || '')}" /></label>
          <label>Telemetry Timeout (ms)<input type="number" data-type="number" data-path="timeouts.telemetryTimeoutMs" value="${esc(timeouts.telemetryTimeoutMs || '')}" /></label>
          <label>PLC Present Warn (ms)<input type="number" data-type="number" data-path="timeouts.plcPresentWarnMs" value="${esc(timeouts.plcPresentWarnMs || '')}" /></label>
          <label>PLC Limits Warn (ms)<input type="number" data-type="number" data-path="timeouts.plcLimitsWarnMs" value="${esc(timeouts.plcLimitsWarnMs || '')}" /></label>
        </div>
      </div>
    </div>
  `;
  bindInputs(target);
}

function renderOcpp() {
  const target = sections.ocpp;
  const ocpp = state.config.ocpp || { Internal: {}, Core: {}, Security: {} };
  target.innerHTML = `
    <div class="section-header">
      <div>
        <p class="eyebrow">OCPP</p>
        <h2>Core profile & endpoints</h2>
        <p class="muted">Identity, URIs, and heartbeat knobs.</p>
      </div>
    </div>
    <div class="grid two stretch">
      <div class="card">
        <div class="card-head">
          <h3>Internal</h3>
        </div>
        <div class="form-grid two">
          <label>Charge Point ID<input data-path="ocpp.Internal.ChargePointId" value="${esc(getByPath(ocpp, 'Internal.ChargePointId', ''))}" /></label>
          <label>Central System URI<input data-path="ocpp.Internal.CentralSystemURI" value="${esc(getByPath(ocpp, 'Internal.CentralSystemURI', ''))}" /></label>
          <label>Vendor<input data-path="ocpp.Internal.ChargePointVendor" value="${esc(getByPath(ocpp, 'Internal.ChargePointVendor', ''))}" /></label>
          <label>Model<input data-path="ocpp.Internal.ChargePointModel" value="${esc(getByPath(ocpp, 'Internal.ChargePointModel', ''))}" /></label>
          <label>Firmware<input data-path="ocpp.Internal.FirmwareVersion" value="${esc(getByPath(ocpp, 'Internal.FirmwareVersion', ''))}" /></label>
        </div>
      </div>
      <div class="card">
        <div class="card-head">
          <h3>Core</h3>
        </div>
        <div class="form-grid two">
          <label>Heartbeat Interval<input type="number" data-type="number" data-path="ocpp.Core.HeartbeatInterval" value="${esc(getByPath(ocpp, 'Core.HeartbeatInterval', ''))}" /></label>
          <label>Meter Sample Interval<input type="number" data-type="number" data-path="ocpp.Core.MeterValueSampleInterval" value="${esc(getByPath(ocpp, 'Core.MeterValueSampleInterval', ''))}" /></label>
          <label>Status Minimum (s)<input type="number" data-type="number" data-path="ocpp.Core.MinimumStatusDuration" value="${esc(getByPath(ocpp, 'Core.MinimumStatusDuration', ''))}" /></label>
          <label>Number Of Connectors<input type="number" data-type="number" data-path="ocpp.Core.NumberOfConnectors" value="${esc(getByPath(ocpp, 'Core.NumberOfConnectors', state.config.connectors ? state.config.connectors.length : ''))}" /></label>
          <label>Authorization Cache<input type="checkbox" data-type="bool" data-path="ocpp.Core.AuthorizationCacheEnabled" ${getByPath(ocpp, 'Core.AuthorizationCacheEnabled') ? 'checked' : ''} /></label>
        </div>
      </div>
    </div>
    <div class="card">
      <div class="card-head">
        <h3>Raw OCPP block</h3>
        <p class="muted small">Direct edit of the inline ocpp object if you need full control.</p>
      </div>
      <textarea id="ocpp-raw" class="code">${esc(state.ocppText)}</textarea>
      <div class="actions right">
        <button class="ghost" id="apply-ocpp">Apply OCPP JSON</button>
      </div>
    </div>
  `;
  bindInputs(target);
  const apply = document.getElementById('apply-ocpp');
  if (apply) {
    apply.onclick = () => {
      const text = document.getElementById('ocpp-raw').value;
      try {
        const parsed = JSON.parse(text);
        state.config.ocpp = parsed;
        state.ocppText = JSON.stringify(parsed, null, 2);
        setDirty(true);
        toast('OCPP block updated', 'success');
      } catch (err) {
        toast('Invalid OCPP JSON: ' + err.message, 'danger');
      }
    };
  }
}

function renderSecurity() {
  const target = sections.security;
  const security = state.config.security || {};
  const certFields = [
    { path: 'security.csmsCaBundle', label: 'CSMS CA Bundle' },
    { path: 'security.moCaBundle', label: 'MO CA Bundle' },
    { path: 'security.v2gCaBundle', label: 'V2G CA Bundle' },
    { path: 'security.clientCertDir', label: 'Client Cert Dir' },
    { path: 'security.clientKeyDir', label: 'Client Key Dir' },
    { path: 'security.seccCertDir', label: 'SECC Cert Dir' },
    { path: 'security.seccKeyDir', label: 'SECC Key Dir' },
  ];
  const pathFields = [
    { path: 'sharePath', label: 'Share Path' },
    { path: 'userConfig', label: 'User Config' },
    { path: 'databaseDir', label: 'Database Dir' },
    { path: 'sqlMigrationsPath', label: 'SQL Migrations' },
    { path: 'messageLogPath', label: 'Message Log Path' },
    { path: 'loggingConfig', label: 'Logging Config' },
  ];
  target.innerHTML = `
    <div class="section-header">
      <div>
        <p class="eyebrow">Security</p>
        <h2>Certificates & storage</h2>
        <p class="muted">Path configuration for PKI material.</p>
      </div>
    </div>
    <div class="grid two stretch">
      ${certFields.map((f) => `
        <div class="card">
          <p class="muted small">${esc(f.label)}</p>
          <input data-path="${f.path}" value="${esc(getByPath(state.config, f.path, ''))}" />
        </div>
      `).join('')}
      <div class="card">
        <div class="card-head">
          <div>
            <p class="eyebrow">Storage</p>
            <h3>Data & logging paths</h3>
          </div>
        </div>
        <div class="form-grid two">
          ${pathFields.map((f) => `
            <label>${esc(f.label)}<input data-path="${f.path}" value="${esc(getByPath(state.config, f.path, ''))}" /></label>
          `).join('')}
        </div>
      </div>
    </div>
  `;
  bindInputs(target);
}

function renderRaw() {
  const target = sections.raw;
  target.innerHTML = `
    <div class="section-header">
      <div>
        <p class="eyebrow">Raw JSON</p>
        <h2>Full charger.json</h2>
        <p class="muted">Paste an existing config or tweak the generated JSON directly.</p>
      </div>
      <div class="actions">
        <button class="ghost" id="refresh-json">Reset view</button>
      </div>
    </div>
    <div class="card">
      <textarea id="raw-json" class="code" spellcheck="false">${esc(JSON.stringify(state.config, null, 2))}</textarea>
      <div class="actions right">
        <button class="ghost" id="apply-raw">Apply JSON</button>
      </div>
    </div>
    <div class="card">
      <div class="card-head">
        <h3>Backups</h3>
        <p class="muted small">Automatic backup is taken before each save.</p>
      </div>
      <div class="history">
        ${(state.history || []).map((h) => `
          <div class="history-row">
            <div>
              <p>${esc(h.name)}</p>
              <p class="muted small">${humanBytes(h.size)} · ${humanTime(h.modified)}</p>
            </div>
            <div class="actions">
              <span class="pill subtle">${esc(h.path)}</span>
              <button class="ghost" data-restore="${esc(h.path)}">Restore</button>
            </div>
          </div>
        `).join('')}
      </div>
    </div>
  `;
  const apply = document.getElementById('apply-raw');
  if (apply) {
    apply.onclick = () => {
      const txt = document.getElementById('raw-json').value;
      try {
        state.config = JSON.parse(txt);
        state.ocppText = JSON.stringify(state.config.ocpp || {}, null, 2);
        setDirty(true);
        renderAll();
        toast('JSON applied', 'success');
      } catch (err) {
        toast('Invalid JSON: ' + err.message, 'danger');
      }
    };
  }
  const refresh = document.getElementById('refresh-json');
  if (refresh) {
    refresh.onclick = () => {
      renderRaw();
    };
  }
  target.querySelectorAll('[data-restore]').forEach((btn) => {
    btn.addEventListener('click', () => {
      const path = btn.dataset.restore;
      restoreBackup(path).catch((err) => toast(err.message, 'danger'));
    });
  });
}

function renderAll() {
  renderOverview();
  renderConnectors();
  renderSlots();
  renderLimits();
  renderOcpp();
  renderSecurity();
  renderRaw();
}

function initNav() {
  document.querySelectorAll('[data-target]').forEach((btn) => {
    btn.addEventListener('click', () => navTo(btn.dataset.target));
  });
}

async function bootstrap() {
  sections.overview = document.getElementById('section-overview');
  sections.connectors = document.getElementById('section-connectors');
  sections.slots = document.getElementById('section-slots');
  sections.limits = document.getElementById('section-limits');
  sections.ocpp = document.getElementById('section-ocpp');
  sections.security = document.getElementById('section-security');
  sections.raw = document.getElementById('section-raw');

  document.getElementById('save-btn').onclick = () => saveConfig().catch((err) => toast(err.message, 'danger'));
  document.getElementById('validate-btn').onclick = () => validateConfig().catch((err) => toast(err.message, 'danger'));
  document.getElementById('backup-btn').onclick = () => backupConfig().catch((err) => toast(err.message, 'danger'));
  document.getElementById('reload-btn').onclick = () => {
    loadConfig().then(() => {
      renderAll();
      toast('Reloaded from disk', 'success');
    }).catch((err) => toast(err.message, 'danger'));
  };

  initNav();
  try {
    await loadConfig();
    await loadHistory();
    renderAll();
    navTo('overview');
  } catch (err) {
    state.lastError = err.message;
    toast(err.message, 'danger');
    const target = document.querySelector('.main');
    if (target) {
      target.innerHTML = `<div class="card"><h3>Failed to load configuration</h3><p class="muted">${esc(err.message)}</p><button id="retry-load">Retry</button></div>`;
      const retry = document.getElementById('retry-load');
      if (retry) {
        retry.onclick = () => bootstrap();
      }
    }
  }

  if (!unloadBound) {
    window.addEventListener('beforeunload', (e) => {
      if (!state.dirty) return;
      e.preventDefault();
      e.returnValue = '';
    });
    unloadBound = true;
  }
}

window.addEventListener('DOMContentLoaded', bootstrap);
