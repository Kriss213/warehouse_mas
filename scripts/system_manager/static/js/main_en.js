const socket = io("/");

const unloadingPointStatus = document.getElementById('unloadingPointStatus');
const dispensersStatus = document.getElementById('dispensersStatus');
const couriersStatus = document.getElementById('couriersStatus');
const generateRequestsBtn = document.getElementById('btn_generate_requests');
const requestTableBody = document.getElementById('requestTableBody');
const requestForm = document.getElementById('requestForm');

// Translation maps
const statusToGui = {
  'idle': 'Free',
  'busy': 'Busy',
  'waiting': 'Waiting',
  'loading': 'Loading',
  'unloading': 'Unloading',
  'en_route': 'En Route',
  'unloading_line': 'In Line for Unloading'
};

const unloadStatus = {
  'idle': 'Waiting',
  'unloading': 'Unloading Packages'
};

const packageStatusToGui = {
  'In Queue': 'In Queue',
  'Unloaded': 'Unloaded',
  'Assigned loader': 'Assigned Loader',
  'Assigned courier': 'Assigned Courier',
  'loading': 'Loading',
  'loaded': 'Loaded',
  'en_route': 'En Route',
  'unloading_line': 'In Line for Unloading',
  'delivered': 'Delivered'
};

const priorityMap = {
  'high': 'High',
  'normal': 'Normal',
  'low': 'Low'
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
  const additionalInfo = load ? `(Load: ${load}/10 kg)` : '';

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
    <span class="${status}"></span> Unloading Point: ${unloadStatus[status]}
  `;
});

// Update loaders
socket.on('loader_status', ({ id, name, status }) => {
  updateStatus(dispensersStatus, id, `Loader ${id}`, status);
});

// Update couriers
socket.on('courier_update', ({ id, name, status, load }) => {
  updateStatus(couriersStatus, id, `Robot ${id}`, status, load);
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
    console.error(`Failed to update request status: ${id}`);
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
    <td>None</td>
    <td>None</td>
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
    status: 'In Queue',
    loader: null,
    courier: null
  };

  addRequestToTable(request);
  socket.emit('new_request', request);
  requestForm.reset();
});

// Generate random values for requests
function getRandomElement(arr) {
  return arr[Math.floor(Math.random() * arr.length)];
}

generateRequestsBtn.addEventListener('click', () => {
  const packageNames = ['Apples', 'Bananas', 'Oranges', 'Bread', 'Milk', 'TV', 'Jacket'];
  const packageTypes = ['Food', 'Drinks', 'Electronics', 'Toys', 'Clothing'];
  const priorities = ['high', 'normal', 'low'];

  const request = {
    id: `REQ-${generateRandomId()}`,
    name: getRandomElement(packageNames),
    type: getRandomElement(packageTypes),
    weight: (Math.random() * 10).toFixed(2),
    priority: getRandomElement(priorities),
    status: 'In Queue',
    loader: null,
    courier: null
  };

  addRequestToTable(request);
  socket.emit('new_request', request);
});
