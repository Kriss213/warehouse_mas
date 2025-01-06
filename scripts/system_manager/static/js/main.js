const socket = io("/");

const unloadingPointStatus = document.getElementById('unloadingPointStatus');
const dispensersStatus = document.getElementById('dispensersStatus');
const couriersStatus = document.getElementById('couriersStatus');
const generateRequestsBtn = document.getElementById('btn_generate_requests');
const requestTableBody = document.getElementById('requestTableBody');
const requestForm = document.getElementById('requestForm');

// Translation maps
const statusToGui = {
  'idle': 'Brīvs',
  'busy': 'Aizņemts',
  'waiting': 'Gaida',
  'loading': 'Uzkraujas',
  'unloading': 'Nokraujas',
  'en_route': 'Ceļā',
  'unloading_line': 'Rindā uz izkraušanu'
};

const unloadStatus = {
  'idle': 'Gaida',
  'unloading': 'Nokrauj paciņas'
};

const packageStatusToGui = {
  'In Queue': 'Rindā',
  'Unloaded': 'Nokrauts',
  'Assigned loader': 'Atrasts uzkrāvējs',
  'Assigned courier': 'Atrasts kurjers',
  'loading': 'Tiek uzkrauts',
  'loaded': 'Uzkrauts',
  'en_route': 'Ceļā',
  'unloading_line': 'Rindā uz izkraušanu',
  'delivered': 'Piegādāts'
};

const priorityMap = {
  'high': 'Augsta',
  'normal': 'Normāla',
  'low': 'Zema'
};

// Helper function to create a status indicator element
function createStatusIndicator(id, name, status, additionalInfo = '') {
  const statusElement = document.createElement('div');
  statusElement.classList.add('status-indicator');
  statusElement.dataset.id = id;
  statusElement.innerHTML = `
    <span class="${status}"></span> ${name}: ${statusToGui[status]} ${additionalInfo}
  `;
  return statusElement;
}

// Update or add a status indicator
function updateStatus(container, id, name, status, load = null) {
  let existingIndicator = container.querySelector(`.status-indicator[data-id="${id}"]`);
  const additionalInfo = load ? `(Krava: ${load}/10 kg)` : '';

  if (existingIndicator) {
    existingIndicator.innerHTML = `
      <span class="${status}"></span> ${name}: ${statusToGui[status]} ${additionalInfo}
    `;
  } else {
    container.appendChild(createStatusIndicator(id, name, status, additionalInfo));
  }
}

// Update unloading point status
socket.on('unloading_point_status', (status) => {
  unloadingPointStatus.innerHTML = `
    <span class="${status}"></span> Nokraušanas punkts: ${unloadStatus[status]}
  `;
});

// Update loaders
socket.on('loader_status', ({ id, name, status }) => {
  updateStatus(dispensersStatus, id, `Uzkrāvējs ${id}`, status);
});

// Update couriers
socket.on('courier_update', ({ id, name, status, load }) => {
  updateStatus(couriersStatus, id, `Robots ${id}`, status, load);
});

// Update package request status
socket.on('request_status', (request) => {
  const { id, status, loader, courier } = request;
  const row = requestTableBody.querySelector(`#${id}`);

  if (row) {
    if (packageStatusToGui[status]) row.cells[5].textContent = packageStatusToGui[status];
    if (loader) row.cells[6].textContent = loader;
    if (courier) row.cells[7].textContent = courier;
  } else {
    console.error(`Neizdevās atjaunināt pieprasījuma statusu: ${id}`);
  }
});

// Generate a random ID
function generateRandomId() {
  return Math.floor(Math.random() * 1000000).toString();
}

// Add a new request to the table
function addRequestToTable(request) {
  const newRow = document.createElement('tr');
  newRow.id = request.id;
  newRow.innerHTML = `
    <td>${request.id}</td>
    <td>${request.name}</td>
    <td>${request.type}</td>
    <td>${request.weight} kg</td>
    <td>${priorityMap[request.priority]}</td>
    <td>${request.status}</td>
    <td>Nav</td>
    <td>Nav</td>
  `;
  requestTableBody.appendChild(newRow);
}

// Handle form submission
requestForm.addEventListener('submit', (event) => {
  event.preventDefault();

  const request = {
    id: `REQ-${generateRandomId()}`,
    name: document.getElementById('packageName').value,
    type: document.getElementById('packageType').value,
    weight: parseFloat(document.getElementById('packageWeight').value),
    priority: document.getElementById('priority').value,
    status: 'Rindā',
    loader: null,
    courier: null
  };

  addRequestToTable(request);
  socket.emit('new_request', request);
  requestForm.reset();
});

// Generate random values for requests
generateRequestsBtn.addEventListener('click', () => {
  const packageNames = ['Āboli', 'Banāni', 'Apelsīni', 'Maize', 'Piens', 'Televizors', 'Jaka'];
  const packageTypes = ['Pārtika', 'Dzērieni', 'Elektronika', 'Rotaļlietas', 'Apģērbs'];
  const priorities = ['high', 'normal', 'low'];

  const request = {
    id: `REQ-${generateRandomId()}`,
    name: getRandomElement(packageNames),
    type: getRandomElement(packageTypes),
    weight: (Math.random() * 10).toFixed(2),
    priority: getRandomElement(priorities),
    status: 'Rindā',
    loader: null,
    courier: null
  };

  addRequestToTable(request);
  socket.emit('new_request', request);
});

function getRandomElement(arr) {
  return arr[Math.floor(Math.random() * arr.length)];
}
